/**
 * @file MLFeatureExtractor.cpp
 * @brief Advanced ML Feature Extractor Implementation
 *
 * Professional implementation providing comprehensive multi-modal feature
 * extraction optimized for banking-grade machine learning algorithms with
 * 3D landmark integration and quality assessment.
 *
 * @version 2.0.0-banking-ml
 * @author Unlook Team - API Architecture Agent
 * @copyright 2025 Unlook. All rights reserved.
 */

#include "unlook/face/MLFeatureExtractor.hpp"
#include "unlook/core/Logger.hpp"
#include "unlook/core/Exception.hpp"

#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/objdetect.hpp>
#include <opencv2/ml.hpp>

#include <algorithm>
#include <cmath>
#include <numeric>
#include <random>
#include <iomanip>
#include <sstream>
#include <fstream>

namespace unlook {
namespace face {

// ============================================================================
// MLFeatureExtractionConfig Implementation
// ============================================================================

bool MLFeatureExtractionConfig::validate() const {
    if (feature_vector_size <= 0 || feature_vector_size > 10000) {
        return false;
    }

    if (quality_threshold < 0.0f || quality_threshold > 1.0f) {
        return false;
    }

    if (banking_quality_threshold < 0.0f || banking_quality_threshold > 1.0f) {
        return false;
    }

    if (max_processing_threads <= 0 || max_processing_threads > 32) {
        return false;
    }

    if (enable_pca_reduction && (pca_dimensions <= 0 || pca_dimensions >= feature_vector_size)) {
        return false;
    }

    const std::vector<std::string> valid_normalization_methods = {"L1", "L2", "MinMax"};
    if (std::find(valid_normalization_methods.begin(), valid_normalization_methods.end(),
                 normalization_method) == valid_normalization_methods.end()) {
        return false;
    }

    return true;
}

std::string MLFeatureExtractionConfig::toString() const {
    std::ostringstream oss;
    oss << "MLFeatureExtractionConfig{";
    oss << "feature_vector_size=" << feature_vector_size << ", ";
    oss << "normalization_method='" << normalization_method << "', ";
    oss << "quality_threshold=" << quality_threshold << ", ";
    oss << "banking_quality_threshold=" << banking_quality_threshold << ", ";
    oss << "max_threads=" << max_processing_threads << ", ";
    oss << "enable_2d=" << (enable_2d_features ? "true" : "false") << ", ";
    oss << "enable_3d=" << (enable_3d_features ? "true" : "false") << ", ";
    oss << "enable_depth=" << (enable_depth_features ? "true" : "false") << ", ";
    oss << "enable_ml=" << (enable_deep_learning_features ? "true" : "false") << ", ";
    oss << "banking_grade=" << (banking_grade_extraction ? "true" : "false");
    oss << "}";
    return oss.str();
}

// ============================================================================
// MLFeatureExtractor Implementation
// ============================================================================

MLFeatureExtractor::MLFeatureExtractor()
    : config_{}
    , initialized_(false)
    , banking_compliance_enabled_(false)
    , total_extractions_(0)
    , successful_extractions_(0)
    , failed_extractions_(0)
    , total_extraction_time_(0.0)
    , cumulative_quality_score_(0.0f)
{
    UNLOOK_LOG_INFO("MLFeatureExtractor: Constructor with default configuration");
}

MLFeatureExtractor::MLFeatureExtractor(const MLFeatureExtractionConfig& config)
    : config_(config)
    , initialized_(false)
    , banking_compliance_enabled_(false)
    , total_extractions_(0)
    , successful_extractions_(0)
    , failed_extractions_(0)
    , total_extraction_time_(0.0)
    , cumulative_quality_score_(0.0f)
{
    UNLOOK_LOG_INFO("MLFeatureExtractor: Constructor with custom configuration");
    UNLOOK_LOG_DEBUG("MLFeatureExtractor: Configuration - " + config_.toString());
}

MLFeatureExtractor::~MLFeatureExtractor() {
    UNLOOK_LOG_INFO("MLFeatureExtractor: Destructor called");
}

bool MLFeatureExtractor::initialize(const AdvancedMLConfig& ml_config) {
    UNLOOK_LOG_INFO("MLFeatureExtractor: Initializing ML feature extractor");

    std::lock_guard<std::mutex> lock(config_mutex_);

    // Validate configuration
    if (!config_.validate()) {
        setLastError("Invalid MLFeatureExtractionConfig provided");
        UNLOOK_LOG_ERROR("MLFeatureExtractor: Invalid configuration: " + config_.toString());
        return false;
    }

    // Update configuration based on ML config
    config_.banking_grade_extraction = ml_config.banking_far_requirement <= 0.001f;
    config_.enable_deep_learning_features = ml_config.enable_deep_learning_features;
    config_.enable_quality_aware_features = ml_config.enable_quality_enhancement;

    // Initialize components
    if (!initializeComponents()) {
        setLastError("Failed to initialize feature extraction components");
        UNLOOK_LOG_ERROR("MLFeatureExtractor: Component initialization failed");
        return false;
    }

    initialized_ = true;
    banking_compliance_enabled_ = config_.banking_grade_extraction;

    UNLOOK_LOG_INFO("MLFeatureExtractor: Initialization completed successfully");
    return true;
}

bool MLFeatureExtractor::isInitialized() const {
    return initialized_.load();
}

FaceResultCode MLFeatureExtractor::extractEnhancedFeatures(const BiometricTemplate& template_data,
                                                          std::vector<float>& enhanced_features) {
    UNLOOK_LOG_DEBUG("MLFeatureExtractor: Extracting enhanced features");

    if (!isInitialized()) {
        setLastError("MLFeatureExtractor not initialized");
        return FaceResultCode::ERROR_SUPERNOVA_ML_ERROR;
    }

    MLFeatureExtractionResult extraction_result;
    FaceResultCode result = extractMLFeatures(template_data, extraction_result);

    if (result == FaceResultCode::SUCCESS) {
        enhanced_features = extraction_result.combined_feature_vector;
    }

    return result;
}

FaceResultCode MLFeatureExtractor::extractMLFeatures(const BiometricTemplate& template_data,
                                                    MLFeatureExtractionResult& extraction_result) {
    UNLOOK_LOG_DEBUG("MLFeatureExtractor: Extracting comprehensive ML features");

    if (!isInitialized()) {
        setLastError("MLFeatureExtractor not initialized");
        return FaceResultCode::ERROR_SUPERNOVA_ML_ERROR;
    }

    auto start_time = std::chrono::high_resolution_clock::now();
    extraction_result = MLFeatureExtractionResult{};

    try {
        // Check cache first
        std::string cache_key = generateCacheKey(template_data, "ml_features_v2");
        {
            std::lock_guard<std::mutex> cache_lock(cache_mutex_);
            auto cache_it = feature_cache_.find(cache_key);
            if (cache_it != feature_cache_.end() && config_.enable_feature_caching) {
                extraction_result.combined_feature_vector = cache_it->second;
                UNLOOK_LOG_DEBUG("MLFeatureExtractor: Using cached features");
                return FaceResultCode::SUCCESS;
            }
        }

        // Step 1: Extract 2D features
        auto preprocessing_start = std::chrono::high_resolution_clock::now();

        if (config_.enable_2d_features && !template_data.face_image.empty()) {
            FaceResultCode result_2d = extract2DFeatures(template_data.face_image,
                                                         template_data.landmarks,
                                                         extraction_result.feature_vector_2d);
            if (result_2d != FaceResultCode::SUCCESS) {
                UNLOOK_LOG_WARNING("MLFeatureExtractor: 2D feature extraction failed");
            }
        }

        // Step 2: Extract 3D features
        if (config_.enable_3d_features && !template_data.landmarks_3d.empty()) {
            FaceResultCode result_3d = extract3DFeatures(template_data.landmarks_3d,
                                                         extraction_result.feature_vector_3d);
            if (result_3d != FaceResultCode::SUCCESS) {
                UNLOOK_LOG_WARNING("MLFeatureExtractor: 3D feature extraction failed");
            }
        }

        // Step 3: Extract depth features
        if (config_.enable_depth_features && !template_data.depth_map.empty()) {
            cv::Rect face_region(0, 0, template_data.depth_map.cols, template_data.depth_map.rows);
            FaceResultCode result_depth = extractDepthFeatures(template_data.depth_map,
                                                              face_region,
                                                              extraction_result.feature_vector_depth);
            if (result_depth != FaceResultCode::SUCCESS) {
                UNLOOK_LOG_WARNING("MLFeatureExtractor: Depth feature extraction failed");
            }
        }

        auto preprocessing_end = std::chrono::high_resolution_clock::now();
        extraction_result.performance_metrics.preprocessing_time_ms =
            std::chrono::duration_cast<std::chrono::microseconds>(preprocessing_end - preprocessing_start).count() / 1000.0;

        // Step 4: Extract texture features
        auto feature_computation_start = std::chrono::high_resolution_clock::now();

        if (config_.enable_texture_features && !template_data.face_image.empty()) {
            cv::Rect face_region(0, 0, template_data.face_image.cols, template_data.face_image.rows);
            FaceResultCode result_texture = extractTextureFeatures(template_data.face_image,
                                                                  face_region,
                                                                  extraction_result.feature_vector_texture);
            if (result_texture != FaceResultCode::SUCCESS) {
                UNLOOK_LOG_WARNING("MLFeatureExtractor: Texture feature extraction failed");
            }
        }

        // Step 5: Extract geometric features
        if (config_.enable_geometric_features) {
            FaceResultCode result_geometric = extractGeometricFeatures(template_data.landmarks,
                                                                      extraction_result.feature_vector_geometric);
            if (result_geometric != FaceResultCode::SUCCESS) {
                UNLOOK_LOG_WARNING("MLFeatureExtractor: Geometric feature extraction failed");
            }
        }

        auto feature_computation_end = std::chrono::high_resolution_clock::now();
        extraction_result.performance_metrics.feature_computation_time_ms =
            std::chrono::duration_cast<std::chrono::microseconds>(feature_computation_end - feature_computation_start).count() / 1000.0;

        // Step 6: Combine feature vectors
        auto postprocessing_start = std::chrono::high_resolution_clock::now();

        std::vector<std::vector<float>> all_features = {
            extraction_result.feature_vector_2d,
            extraction_result.feature_vector_3d,
            extraction_result.feature_vector_depth,
            extraction_result.feature_vector_texture,
            extraction_result.feature_vector_geometric
        };

        FaceResultCode combine_result = combineFeatureVectors(all_features,
                                                             "concatenate",
                                                             extraction_result.combined_feature_vector);
        if (combine_result != FaceResultCode::SUCCESS) {
            setLastError("Failed to combine feature vectors");
            return combine_result;
        }

        // Step 7: Apply normalization
        if (config_.normalize_features) {
            FaceResultCode normalize_result = normalizeFeatures(extraction_result.combined_feature_vector,
                                                               config_.normalization_method);
            if (normalize_result != FaceResultCode::SUCCESS) {
                UNLOOK_LOG_WARNING("MLFeatureExtractor: Feature normalization failed");
            }
        }

        // Step 8: Apply deep learning enhancement
        if (config_.enable_deep_learning_features) {
            FaceResultCode dl_result = enhanceWithDeepLearning(extraction_result.combined_feature_vector,
                                                              extraction_result.deep_learning_features);
            if (dl_result == FaceResultCode::SUCCESS) {
                extraction_result.combined_feature_vector = extraction_result.deep_learning_features;
            } else {
                UNLOOK_LOG_WARNING("MLFeatureExtractor: Deep learning enhancement failed");
            }
        }

        auto postprocessing_end = std::chrono::high_resolution_clock::now();
        extraction_result.performance_metrics.postprocessing_time_ms =
            std::chrono::duration_cast<std::chrono::microseconds>(postprocessing_end - postprocessing_start).count() / 1000.0;

        // Step 9: Quality assessment
        float quality_score = assessFeatureQuality(extraction_result.combined_feature_vector,
                                                   template_data,
                                                   extraction_result.quality_metrics);

        // Step 10: Banking compliance validation
        if (banking_compliance_enabled_.load()) {
            extraction_result.quality_metrics.banking_grade_score = quality_score;
            extraction_result.quality_metrics.meets_banking_requirements =
                quality_score >= config_.banking_quality_threshold;

            if (!extraction_result.quality_metrics.meets_banking_requirements) {
                setLastError("Features do not meet banking quality requirements");
                UNLOOK_LOG_WARNING("MLFeatureExtractor: Banking quality requirements not met");
            }
        }

        // Update performance metrics
        auto end_time = std::chrono::high_resolution_clock::now();
        extraction_result.performance_metrics.extraction_time_ms =
            std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count() / 1000.0;

        extraction_result.performance_metrics.features_extracted = extraction_result.combined_feature_vector.size();
        extraction_result.extraction_method = "multi_modal_ml_v2";

        // Cache result
        if (config_.enable_feature_caching) {
            std::lock_guard<std::mutex> cache_lock(cache_mutex_);
            if (feature_cache_.size() < max_cache_size_) {
                feature_cache_[cache_key] = extraction_result.combined_feature_vector;
            }
        }

        // Update statistics
        updateStatistics(extraction_result.performance_metrics.extraction_time_ms, true, quality_score);

        UNLOOK_LOG_DEBUG("MLFeatureExtractor: Feature extraction completed successfully in " +
                        std::to_string(extraction_result.performance_metrics.extraction_time_ms) + " ms");

        return FaceResultCode::SUCCESS;

    } catch (const std::exception& e) {
        setLastError("Feature extraction exception: " + std::string(e.what()));
        UNLOOK_LOG_ERROR("MLFeatureExtractor: Exception during extraction: " + std::string(e.what()));

        // Update statistics for failure
        auto end_time = std::chrono::high_resolution_clock::now();
        double extraction_time = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count() / 1000.0;
        updateStatistics(extraction_time, false);

        return FaceResultCode::ERROR_TEMPLATE_GENERATION_FAILED;
    }
}

float MLFeatureExtractor::assessTemplateQuality(const BiometricTemplate& template_data,
                                               std::map<std::string, float>& quality_report) {
    UNLOOK_LOG_DEBUG("MLFeatureExtractor: Assessing template quality");

    quality_report.clear();

    if (!isInitialized()) {
        setLastError("MLFeatureExtractor not initialized");
        return 0.0f;
    }

    try {
        // Basic quality assessments
        float image_quality = 0.8f; // Default score
        if (!template_data.face_image.empty()) {
            cv::Scalar mean, stddev;
            cv::meanStdDev(template_data.face_image, mean, stddev);

            // Assess image sharpness using Laplacian variance
            cv::Mat gray, laplacian;
            if (template_data.face_image.channels() == 3) {
                cv::cvtColor(template_data.face_image, gray, cv::COLOR_BGR2GRAY);
            } else {
                gray = template_data.face_image.clone();
            }

            cv::Laplacian(gray, laplacian, CV_64F);
            cv::Scalar laplacian_mean, laplacian_stddev;
            cv::meanStdDev(laplacian, laplacian_mean, laplacian_stddev);

            float sharpness = laplacian_stddev[0] * laplacian_stddev[0];
            image_quality = std::min(1.0f, sharpness / 1000.0f); // Normalize sharpness

            quality_report["image_sharpness"] = image_quality;
            quality_report["mean_intensity"] = static_cast<float>(mean[0]);
            quality_report["intensity_variance"] = static_cast<float>(stddev[0] * stddev[0]);
        }

        // Landmark quality assessment
        float landmark_quality = 0.7f; // Default score
        if (!template_data.landmarks.points.empty()) {
            // Calculate landmark stability/consistency
            std::vector<float> distances;
            for (size_t i = 1; i < template_data.landmarks.points.size(); ++i) {
                cv::Point2f diff = template_data.landmarks.points[i] - template_data.landmarks.points[i-1];
                distances.push_back(cv::norm(diff));
            }

            if (!distances.empty()) {
                float mean_distance = std::accumulate(distances.begin(), distances.end(), 0.0f) / distances.size();
                float variance = 0.0f;
                for (float dist : distances) {
                    variance += (dist - mean_distance) * (dist - mean_distance);
                }
                variance /= distances.size();

                landmark_quality = 1.0f / (1.0f + variance / 100.0f); // Normalize variance impact
                quality_report["landmark_stability"] = landmark_quality;
                quality_report["landmark_mean_distance"] = mean_distance;
                quality_report["landmark_variance"] = variance;
            }
        }

        // Depth quality assessment
        float depth_quality = 0.8f; // Default score
        if (!template_data.depth_map.empty()) {
            cv::Scalar depth_mean, depth_stddev;
            cv::meanStdDev(template_data.depth_map, depth_mean, depth_stddev);

            // Good depth quality means reasonable depth values with good variance
            float depth_range = depth_stddev[0];
            depth_quality = std::min(1.0f, depth_range / 50.0f); // Normalize depth range

            quality_report["depth_mean"] = static_cast<float>(depth_mean[0]);
            quality_report["depth_variance"] = static_cast<float>(depth_stddev[0] * depth_stddev[0]);
            quality_report["depth_quality"] = depth_quality;
        }

        // Feature vector quality
        float feature_quality = 0.8f;
        if (!template_data.feature_vector.empty()) {
            // Check for feature vector completeness and distribution
            float feature_mean = std::accumulate(template_data.feature_vector.begin(),
                                               template_data.feature_vector.end(), 0.0f) /
                                template_data.feature_vector.size();

            float feature_variance = 0.0f;
            for (float val : template_data.feature_vector) {
                feature_variance += (val - feature_mean) * (val - feature_mean);
            }
            feature_variance /= template_data.feature_vector.size();

            // Good feature quality means appropriate variance (not too flat, not too noisy)
            feature_quality = 1.0f / (1.0f + std::abs(feature_variance - 0.5f));

            quality_report["feature_mean"] = feature_mean;
            quality_report["feature_variance"] = feature_variance;
            quality_report["feature_completeness"] = static_cast<float>(template_data.feature_vector.size()) / config_.feature_vector_size;
        }

        // Template-level quality assessment
        float template_completeness = 0.0f;
        int components_present = 0;
        int total_components = 5; // face_image, landmarks, depth_map, feature_vector, metadata

        if (!template_data.face_image.empty()) components_present++;
        if (!template_data.landmarks.points.empty()) components_present++;
        if (!template_data.depth_map.empty()) components_present++;
        if (!template_data.feature_vector.empty()) components_present++;
        if (!template_data.metadata.empty()) components_present++;

        template_completeness = static_cast<float>(components_present) / total_components;

        quality_report["template_completeness"] = template_completeness;
        quality_report["components_present"] = static_cast<float>(components_present);

        // Overall quality score (weighted combination)
        float overall_quality = (image_quality * 0.25f +
                                landmark_quality * 0.25f +
                                depth_quality * 0.20f +
                                feature_quality * 0.20f +
                                template_completeness * 0.10f);

        quality_report["overall_quality"] = overall_quality;
        quality_report["image_quality"] = image_quality;
        quality_report["landmark_quality"] = landmark_quality;
        quality_report["depth_quality"] = depth_quality;
        quality_report["feature_quality"] = feature_quality;

        // Banking-specific quality assessments
        if (banking_compliance_enabled_.load()) {
            quality_report["banking_grade_ready"] = overall_quality >= config_.banking_quality_threshold ? 1.0f : 0.0f;
            quality_report["banking_quality_threshold"] = config_.banking_quality_threshold;
        }

        UNLOOK_LOG_DEBUG("MLFeatureExtractor: Quality assessment completed - overall score: " +
                        std::to_string(overall_quality));

        return overall_quality;

    } catch (const std::exception& e) {
        setLastError("Quality assessment exception: " + std::string(e.what()));
        UNLOOK_LOG_ERROR("MLFeatureExtractor: Quality assessment exception: " + std::string(e.what()));
        return 0.0f;
    }
}

void MLFeatureExtractor::getExtractionStatistics(size_t& total_extractions,
                                                size_t& successful_extractions,
                                                size_t& failed_extractions,
                                                double& avg_extraction_time,
                                                float& avg_quality_score) const {
    std::lock_guard<std::mutex> lock(stats_mutex_);

    total_extractions = total_extractions_.load();
    successful_extractions = successful_extractions_.load();
    failed_extractions = failed_extractions_.load();

    if (total_extractions > 0) {
        avg_extraction_time = total_extraction_time_.load() / total_extractions;
        avg_quality_score = cumulative_quality_score_.load() / total_extractions;
    } else {
        avg_extraction_time = 0.0;
        avg_quality_score = 0.0f;
    }
}

std::string MLFeatureExtractor::getLastError() const {
    std::lock_guard<std::mutex> lock(error_mutex_);
    return last_error_;
}

// ============================================================================
// Private Implementation Methods
// ============================================================================

bool MLFeatureExtractor::initializeComponents() {
    UNLOOK_LOG_INFO("MLFeatureExtractor: Initializing feature extraction components");

    try {
        // Initialize HOG descriptor for 2D features
        if (config_.enable_2d_features) {
            hog_descriptor_ = std::make_unique<cv::HOGDescriptor>();
            hog_descriptor_->setSVMDetector(cv::HOGDescriptor::getDefaultPeopleDetector());
        }

        // Initialize SIFT detector for texture features
        if (config_.enable_texture_features) {
            sift_detector_ = cv::SIFT::create(0, 3, 0.04, 10, 1.6);
        }

        // Initialize feature templates for geometric features
        if (config_.enable_geometric_features) {
            // Pre-compute geometric feature templates
            feature_templates_.resize(10); // Reserve space for templates
        }

        UNLOOK_LOG_INFO("MLFeatureExtractor: All components initialized successfully");
        return true;

    } catch (const std::exception& e) {
        UNLOOK_LOG_ERROR("MLFeatureExtractor: Component initialization exception: " +
                        std::string(e.what()));
        return false;
    }
}

FaceResultCode MLFeatureExtractor::extract2DFeatures(const cv::Mat& image,
                                                    const FacialLandmarks& landmarks,
                                                    std::vector<float>& features_2d) {
    features_2d.clear();

    try {
        cv::Mat gray_image;
        if (image.channels() == 3) {
            cv::cvtColor(image, gray_image, cv::COLOR_BGR2GRAY);
        } else {
            gray_image = image.clone();
        }

        // Extract HOG features
        if (hog_descriptor_) {
            std::vector<float> hog_features;
            std::vector<cv::Point> locations;
            hog_descriptor_->compute(gray_image, hog_features, cv::Size(8, 8), cv::Size(0, 0), locations);
            features_2d.insert(features_2d.end(), hog_features.begin(), hog_features.end());
        }

        // Extract LBP (Local Binary Patterns) features
        cv::Mat lbp_image;
        lbp_image = cv::Mat::zeros(gray_image.size(), CV_8UC1);

        for (int i = 1; i < gray_image.rows - 1; i++) {
            for (int j = 1; j < gray_image.cols - 1; j++) {
                uint8_t center = gray_image.at<uint8_t>(i, j);
                uint8_t lbp_value = 0;

                // Compute LBP value
                lbp_value |= (gray_image.at<uint8_t>(i-1, j-1) >= center) << 7;
                lbp_value |= (gray_image.at<uint8_t>(i-1, j) >= center) << 6;
                lbp_value |= (gray_image.at<uint8_t>(i-1, j+1) >= center) << 5;
                lbp_value |= (gray_image.at<uint8_t>(i, j+1) >= center) << 4;
                lbp_value |= (gray_image.at<uint8_t>(i+1, j+1) >= center) << 3;
                lbp_value |= (gray_image.at<uint8_t>(i+1, j) >= center) << 2;
                lbp_value |= (gray_image.at<uint8_t>(i+1, j-1) >= center) << 1;
                lbp_value |= (gray_image.at<uint8_t>(i, j-1) >= center) << 0;

                lbp_image.at<uint8_t>(i, j) = lbp_value;
            }
        }

        // Create histogram of LBP values
        std::vector<int> lbp_histogram(256, 0);
        for (int i = 0; i < lbp_image.rows; i++) {
            for (int j = 0; j < lbp_image.cols; j++) {
                lbp_histogram[lbp_image.at<uint8_t>(i, j)]++;
            }
        }

        // Normalize and add LBP histogram to features
        int total_pixels = lbp_image.rows * lbp_image.cols;
        for (int count : lbp_histogram) {
            features_2d.push_back(static_cast<float>(count) / total_pixels);
        }

        return FaceResultCode::SUCCESS;

    } catch (const std::exception& e) {
        setLastError("2D feature extraction failed: " + std::string(e.what()));
        return FaceResultCode::ERROR_TEMPLATE_GENERATION_FAILED;
    }
}

FaceResultCode MLFeatureExtractor::extract3DFeatures(const std::vector<cv::Point3f>& landmarks_3d,
                                                    std::vector<float>& features_3d) {
    features_3d.clear();

    if (landmarks_3d.empty()) {
        return FaceResultCode::ERROR_INSUFFICIENT_LANDMARKS;
    }

    try {
        // Extract geometric features from 3D landmarks
        cv::Point3f centroid(0.0f, 0.0f, 0.0f);
        for (const auto& point : landmarks_3d) {
            centroid.x += point.x;
            centroid.y += point.y;
            centroid.z += point.z;
        }
        centroid.x /= landmarks_3d.size();
        centroid.y /= landmarks_3d.size();
        centroid.z /= landmarks_3d.size();

        // Extract distances from centroid
        for (const auto& point : landmarks_3d) {
            cv::Point3f diff = point - centroid;
            float distance = std::sqrt(diff.x * diff.x + diff.y * diff.y + diff.z * diff.z);
            features_3d.push_back(distance);
        }

        // Extract pairwise distances between key landmarks
        if (landmarks_3d.size() >= 68) { // Assuming 68-point landmarks
            // Eye-to-eye distance
            cv::Point3f left_eye = landmarks_3d[36];  // Left eye corner
            cv::Point3f right_eye = landmarks_3d[45]; // Right eye corner
            float eye_distance = cv::norm(right_eye - left_eye);
            features_3d.push_back(eye_distance);

            // Nose tip to chin distance
            cv::Point3f nose_tip = landmarks_3d[30];  // Nose tip
            cv::Point3f chin = landmarks_3d[8];       // Chin
            float nose_chin_distance = cv::norm(chin - nose_tip);
            features_3d.push_back(nose_chin_distance);

            // Face width (left to right cheek)
            cv::Point3f left_cheek = landmarks_3d[0];  // Left cheek
            cv::Point3f right_cheek = landmarks_3d[16]; // Right cheek
            float face_width = cv::norm(right_cheek - left_cheek);
            features_3d.push_back(face_width);
        }

        // Extract facial symmetry features
        if (landmarks_3d.size() >= 68) {
            float symmetry_score = 0.0f;
            std::vector<std::pair<int, int>> symmetry_pairs = {
                {0, 16}, {1, 15}, {2, 14}, {3, 13}, {4, 12}, {5, 11}, {6, 10}, {7, 9},  // Face outline
                {17, 26}, {18, 25}, {19, 24}, {20, 23}, {21, 22},  // Eyebrows
                {36, 45}, {37, 44}, {38, 43}, {39, 42}, {40, 47}, {41, 46}  // Eyes
            };

            for (const auto& pair : symmetry_pairs) {
                cv::Point3f left_point = landmarks_3d[pair.first];
                cv::Point3f right_point = landmarks_3d[pair.second];

                // Mirror right point across face center
                cv::Point3f mirrored_right = right_point;
                mirrored_right.x = 2 * centroid.x - right_point.x;

                float distance = cv::norm(left_point - mirrored_right);
                symmetry_score += distance;
            }

            symmetry_score /= symmetry_pairs.size();
            features_3d.push_back(symmetry_score);
        }

        return FaceResultCode::SUCCESS;

    } catch (const std::exception& e) {
        setLastError("3D feature extraction failed: " + std::string(e.what()));
        return FaceResultCode::ERROR_TEMPLATE_GENERATION_FAILED;
    }
}

FaceResultCode MLFeatureExtractor::extractDepthFeatures(const cv::Mat& depth_map,
                                                       const cv::Rect& face_region,
                                                       std::vector<float>& depth_features) {
    depth_features.clear();

    if (depth_map.empty()) {
        return FaceResultCode::ERROR_DEPTH_DATA_INVALID;
    }

    try {
        cv::Mat roi = depth_map(face_region);

        // Basic depth statistics
        cv::Scalar mean_depth, stddev_depth;
        cv::meanStdDev(roi, mean_depth, stddev_depth);
        depth_features.push_back(static_cast<float>(mean_depth[0]));
        depth_features.push_back(static_cast<float>(stddev_depth[0]));

        // Depth histogram features
        double min_val, max_val;
        cv::minMaxLoc(roi, &min_val, &max_val);

        const int histogram_bins = 32;
        std::vector<int> depth_histogram(histogram_bins, 0);

        for (int i = 0; i < roi.rows; i++) {
            for (int j = 0; j < roi.cols; j++) {
                float depth_val = roi.at<float>(i, j);
                if (depth_val > 0) { // Valid depth value
                    int bin = static_cast<int>((depth_val - min_val) / (max_val - min_val) * (histogram_bins - 1));
                    bin = std::max(0, std::min(histogram_bins - 1, bin));
                    depth_histogram[bin]++;
                }
            }
        }

        // Normalize and add histogram to features
        int total_valid_pixels = std::accumulate(depth_histogram.begin(), depth_histogram.end(), 0);
        if (total_valid_pixels > 0) {
            for (int count : depth_histogram) {
                depth_features.push_back(static_cast<float>(count) / total_valid_pixels);
            }
        }

        // Depth gradient features
        cv::Mat grad_x, grad_y;
        cv::Sobel(roi, grad_x, CV_32F, 1, 0, 3);
        cv::Sobel(roi, grad_y, CV_32F, 0, 1, 3);

        cv::Mat gradient_magnitude;
        cv::magnitude(grad_x, grad_y, gradient_magnitude);

        cv::Scalar mean_gradient, stddev_gradient;
        cv::meanStdDev(gradient_magnitude, mean_gradient, stddev_gradient);
        depth_features.push_back(static_cast<float>(mean_gradient[0]));
        depth_features.push_back(static_cast<float>(stddev_gradient[0]));

        return FaceResultCode::SUCCESS;

    } catch (const std::exception& e) {
        setLastError("Depth feature extraction failed: " + std::string(e.what()));
        return FaceResultCode::ERROR_DEPTH_DATA_INVALID;
    }
}

FaceResultCode MLFeatureExtractor::extractTextureFeatures(const cv::Mat& image,
                                                         const cv::Rect& face_region,
                                                         std::vector<float>& texture_features) {
    texture_features.clear();

    try {
        cv::Mat roi = image(face_region);
        cv::Mat gray_roi;

        if (roi.channels() == 3) {
            cv::cvtColor(roi, gray_roi, cv::COLOR_BGR2GRAY);
        } else {
            gray_roi = roi.clone();
        }

        // SIFT features
        if (sift_detector_) {
            std::vector<cv::KeyPoint> keypoints;
            cv::Mat descriptors;
            sift_detector_->detectAndCompute(gray_roi, cv::noArray(), keypoints, descriptors);

            if (!descriptors.empty()) {
                // Compute statistical measures of SIFT descriptors
                cv::Scalar mean_desc, stddev_desc;
                cv::meanStdDev(descriptors, mean_desc, stddev_desc);
                texture_features.push_back(static_cast<float>(mean_desc[0]));
                texture_features.push_back(static_cast<float>(stddev_desc[0]));
                texture_features.push_back(static_cast<float>(keypoints.size()));
            } else {
                texture_features.insert(texture_features.end(), 3, 0.0f); // Add zeros if no keypoints
            }
        }

        // Gabor filter responses
        const int num_orientations = 8;
        const int num_scales = 4;

        for (int scale = 0; scale < num_scales; scale++) {
            for (int orientation = 0; orientation < num_orientations; orientation++) {
                cv::Mat kernel = cv::getGaborKernel(cv::Size(31, 31),
                                                   2 + scale,
                                                   orientation * CV_PI / num_orientations,
                                                   2 * CV_PI, 0.5, 0, CV_32F);

                cv::Mat filtered;
                cv::filter2D(gray_roi, filtered, CV_32F, kernel);

                cv::Scalar mean_response, stddev_response;
                cv::meanStdDev(filtered, mean_response, stddev_response);
                texture_features.push_back(static_cast<float>(mean_response[0]));
                texture_features.push_back(static_cast<float>(stddev_response[0]));
            }
        }

        return FaceResultCode::SUCCESS;

    } catch (const std::exception& e) {
        setLastError("Texture feature extraction failed: " + std::string(e.what()));
        return FaceResultCode::ERROR_TEMPLATE_GENERATION_FAILED;
    }
}

FaceResultCode MLFeatureExtractor::extractGeometricFeatures(const FacialLandmarks& landmarks,
                                                           std::vector<float>& geometric_features) {
    geometric_features.clear();

    if (landmarks.points.empty()) {
        return FaceResultCode::ERROR_INSUFFICIENT_LANDMARKS;
    }

    try {
        // Compute centroid
        cv::Point2f centroid(0.0f, 0.0f);
        for (const auto& point : landmarks.points) {
            centroid.x += point.x;
            centroid.y += point.y;
        }
        centroid.x /= landmarks.points.size();
        centroid.y /= landmarks.points.size();

        // Extract relative positions from centroid
        for (const auto& point : landmarks.points) {
            geometric_features.push_back(point.x - centroid.x);
            geometric_features.push_back(point.y - centroid.y);
        }

        // Extract geometric ratios and angles (for 68-point landmarks)
        if (landmarks.points.size() >= 68) {
            // Eye aspect ratio
            cv::Point2f left_eye_left = landmarks.points[36];
            cv::Point2f left_eye_right = landmarks.points[39];
            cv::Point2f left_eye_top = landmarks.points[37];
            cv::Point2f left_eye_bottom = landmarks.points[41];

            float eye_width = cv::norm(left_eye_right - left_eye_left);
            float eye_height = cv::norm(left_eye_top - left_eye_bottom);
            float eye_aspect_ratio = eye_height / (eye_width + 1e-6f);
            geometric_features.push_back(eye_aspect_ratio);

            // Mouth aspect ratio
            cv::Point2f mouth_left = landmarks.points[48];
            cv::Point2f mouth_right = landmarks.points[54];
            cv::Point2f mouth_top = landmarks.points[51];
            cv::Point2f mouth_bottom = landmarks.points[57];

            float mouth_width = cv::norm(mouth_right - mouth_left);
            float mouth_height = cv::norm(mouth_top - mouth_bottom);
            float mouth_aspect_ratio = mouth_height / (mouth_width + 1e-6f);
            geometric_features.push_back(mouth_aspect_ratio);

            // Face orientation angles
            cv::Point2f nose_tip = landmarks.points[30];
            cv::Point2f chin = landmarks.points[8];
            cv::Point2f forehead = landmarks.points[27];

            cv::Vec2f face_vertical = cv::Vec2f(chin.x - forehead.x, chin.y - forehead.y);
            float face_angle = std::atan2(face_vertical[1], face_vertical[0]);
            geometric_features.push_back(face_angle);
        }

        return FaceResultCode::SUCCESS;

    } catch (const std::exception& e) {
        setLastError("Geometric feature extraction failed: " + std::string(e.what()));
        return FaceResultCode::ERROR_LANDMARK_DETECTION_FAILED;
    }
}

FaceResultCode MLFeatureExtractor::combineFeatureVectors(const std::vector<std::vector<float>>& feature_vectors,
                                                        const std::string& combination_method,
                                                        std::vector<float>& combined_features) {
    combined_features.clear();

    if (feature_vectors.empty()) {
        return FaceResultCode::ERROR_TEMPLATE_GENERATION_FAILED;
    }

    try {
        if (combination_method == "concatenate") {
            // Simple concatenation
            for (const auto& feature_vector : feature_vectors) {
                combined_features.insert(combined_features.end(), feature_vector.begin(), feature_vector.end());
            }
        } else if (combination_method == "weighted") {
            // Weighted combination (equal weights for now)
            std::vector<float> weights = {0.2f, 0.2f, 0.2f, 0.2f, 0.2f}; // Equal weights

            size_t max_size = 0;
            for (const auto& feature_vector : feature_vectors) {
                max_size = std::max(max_size, feature_vector.size());
            }

            combined_features.resize(max_size, 0.0f);

            for (size_t i = 0; i < feature_vectors.size() && i < weights.size(); ++i) {
                const auto& feature_vector = feature_vectors[i];
                float weight = weights[i];

                for (size_t j = 0; j < feature_vector.size() && j < max_size; ++j) {
                    combined_features[j] += feature_vector[j] * weight;
                }
            }
        } else {
            // Default to concatenation
            for (const auto& feature_vector : feature_vectors) {
                combined_features.insert(combined_features.end(), feature_vector.begin(), feature_vector.end());
            }
        }

        return FaceResultCode::SUCCESS;

    } catch (const std::exception& e) {
        setLastError("Feature combination failed: " + std::string(e.what()));
        return FaceResultCode::ERROR_TEMPLATE_GENERATION_FAILED;
    }
}

FaceResultCode MLFeatureExtractor::normalizeFeatures(std::vector<float>& features,
                                                    const std::string& method) {
    if (features.empty()) {
        return FaceResultCode::SUCCESS;
    }

    try {
        if (method == "L2") {
            // L2 normalization
            float norm = 0.0f;
            for (float val : features) {
                norm += val * val;
            }
            norm = std::sqrt(norm);

            if (norm > 1e-6f) {
                for (float& val : features) {
                    val /= norm;
                }
            }
        } else if (method == "L1") {
            // L1 normalization
            float norm = 0.0f;
            for (float val : features) {
                norm += std::abs(val);
            }

            if (norm > 1e-6f) {
                for (float& val : features) {
                    val /= norm;
                }
            }
        } else if (method == "MinMax") {
            // MinMax normalization
            auto minmax = std::minmax_element(features.begin(), features.end());
            float min_val = *minmax.first;
            float max_val = *minmax.second;
            float range = max_val - min_val;

            if (range > 1e-6f) {
                for (float& val : features) {
                    val = (val - min_val) / range;
                }
            }
        }

        return FaceResultCode::SUCCESS;

    } catch (const std::exception& e) {
        setLastError("Feature normalization failed: " + std::string(e.what()));
        return FaceResultCode::ERROR_TEMPLATE_GENERATION_FAILED;
    }
}

FaceResultCode MLFeatureExtractor::enhanceWithDeepLearning(const std::vector<float>& input_features,
                                                          std::vector<float>& enhanced_features) {
    // Placeholder for deep learning enhancement
    // In a real implementation, this would use a trained neural network
    enhanced_features = input_features;

    // Simple enhancement: apply non-linear transformation
    for (float& val : enhanced_features) {
        val = std::tanh(val); // Apply tanh activation
    }

    return FaceResultCode::SUCCESS;
}

float MLFeatureExtractor::assessFeatureQuality(const std::vector<float>& features,
                                              const BiometricTemplate& template_data,
                                              MLFeatureExtractionResult::QualityMetrics& quality_metrics) {
    if (features.empty()) {
        quality_metrics = MLFeatureExtractionResult::QualityMetrics{};
        return 0.0f;
    }

    try {
        // Feature completeness
        quality_metrics.feature_completeness = static_cast<float>(features.size()) / config_.feature_vector_size;

        // Feature consistency (check for NaN or infinite values)
        size_t valid_features = 0;
        for (float val : features) {
            if (std::isfinite(val)) {
                valid_features++;
            }
        }
        quality_metrics.feature_consistency = static_cast<float>(valid_features) / features.size();

        // Feature distribution quality
        float mean = std::accumulate(features.begin(), features.end(), 0.0f) / features.size();
        float variance = 0.0f;
        for (float val : features) {
            variance += (val - mean) * (val - mean);
        }
        variance /= features.size();

        // Good features should have reasonable variance (not too flat, not too noisy)
        float distribution_quality = 1.0f / (1.0f + std::abs(variance - 0.5f));
        quality_metrics.extraction_confidence = distribution_quality;

        // Overall quality score
        quality_metrics.overall_quality_score = (quality_metrics.feature_completeness * 0.4f +
                                                quality_metrics.feature_consistency * 0.4f +
                                                quality_metrics.extraction_confidence * 0.2f);

        // Banking-specific quality assessment
        if (banking_compliance_enabled_.load()) {
            quality_metrics.banking_grade_score = quality_metrics.overall_quality_score;
            quality_metrics.meets_banking_requirements =
                quality_metrics.banking_grade_score >= config_.banking_quality_threshold;
        }

        return quality_metrics.overall_quality_score;

    } catch (const std::exception& e) {
        setLastError("Quality assessment failed: " + std::string(e.what()));
        return 0.0f;
    }
}

void MLFeatureExtractor::updateStatistics(double extraction_time, bool success, float quality_score) {
    std::lock_guard<std::mutex> lock(stats_mutex_);

    total_extractions_++;
    total_extraction_time_ += extraction_time;

    if (success) {
        successful_extractions_++;
        if (quality_score >= 0.0f) {
            cumulative_quality_score_ += quality_score;
        }
    } else {
        failed_extractions_++;
    }
}

std::string MLFeatureExtractor::generateCacheKey(const BiometricTemplate& template_data,
                                                const std::string& config_hash) {
    std::ostringstream oss;
    oss << "template_" << template_data.template_id
        << "_config_" << config_hash
        << "_version_" << template_data.version;
    return oss.str();
}

void MLFeatureExtractor::setLastError(const std::string& error_message) {
    std::lock_guard<std::mutex> lock(error_mutex_);
    last_error_ = error_message;
    error_history_.push_back(error_message);

    // Keep only last 100 errors
    if (error_history_.size() > 100) {
        error_history_.erase(error_history_.begin());
    }
}

} // namespace face
} // namespace unlook