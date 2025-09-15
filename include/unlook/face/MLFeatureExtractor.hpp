#pragma once

/**
 * @file MLFeatureExtractor.hpp
 * @brief Advanced ML Feature Extractor for Supernova Banking Algorithms
 *
 * Professional feature extraction implementation providing comprehensive
 * multi-modal feature extraction optimized for banking-grade machine learning
 * algorithms with 3D landmark integration and quality assessment.
 *
 * @version 2.0.0-banking-ml
 * @author Unlook Team - API Architecture Agent
 * @copyright 2025 Unlook. All rights reserved.
 */

#include "unlook/face/FaceTypes.hpp"
#include "unlook/face/SupernovaMLClient.hpp"
#include "unlook/core/types.hpp"
#include "unlook/core/Logger.hpp"

#include <opencv2/opencv.hpp>
#include <memory>
#include <vector>
#include <string>
#include <map>
#include <mutex>
#include <atomic>
#include <array>

namespace unlook {
namespace face {

/**
 * @brief ML feature extraction configuration for advanced algorithms
 */
struct MLFeatureExtractionConfig {
    // Core feature extraction settings
    bool enable_2d_features = true;                ///< Enable 2D facial features
    bool enable_3d_features = true;                ///< Enable 3D landmark features
    bool enable_depth_features = true;             ///< Enable depth-based features
    bool enable_texture_features = true;           ///< Enable texture analysis
    bool enable_geometric_features = true;         ///< Enable geometric features

    // Advanced ML features
    bool enable_deep_learning_features = true;     ///< Enable DL-based features
    bool enable_multi_scale_features = true;       ///< Enable multi-scale analysis
    bool enable_temporal_features = true;          ///< Enable temporal consistency
    bool enable_quality_aware_features = true;     ///< Enable quality-aware extraction

    // Feature vector settings
    int feature_vector_size = 2048;                ///< Target feature vector size
    bool normalize_features = true;                ///< Normalize feature vectors
    std::string normalization_method = "L2";       ///< L1, L2, or MinMax normalization
    bool enable_pca_reduction = false;             ///< Enable PCA dimensionality reduction
    int pca_dimensions = 512;                      ///< PCA target dimensions

    // Quality assessment settings
    bool enable_quality_scoring = true;            ///< Enable quality scoring
    bool enable_defect_detection = true;           ///< Enable defect detection
    float quality_threshold = 0.7f;                ///< Minimum quality threshold
    bool reject_low_quality = false;               ///< Reject low quality features

    // Performance settings
    bool enable_gpu_acceleration = false;          ///< Enable GPU acceleration
    bool enable_simd_optimization = true;          ///< Enable SIMD optimization
    int max_processing_threads = 4;                ///< Maximum processing threads
    bool enable_feature_caching = true;            ///< Enable feature caching

    // Banking-specific settings
    bool banking_grade_extraction = false;         ///< Enable banking-grade extraction
    bool require_liveness_features = true;         ///< Require liveness-aware features
    bool enable_anti_spoofing_features = true;     ///< Enable anti-spoofing features
    float banking_quality_threshold = 0.85f;       ///< Banking quality threshold

    bool validate() const;
    std::string toString() const;
};

/**
 * @brief Comprehensive feature extraction result
 */
struct MLFeatureExtractionResult {
    // Core feature vectors
    std::vector<float> feature_vector_2d;          ///< 2D facial features
    std::vector<float> feature_vector_3d;          ///< 3D landmark features
    std::vector<float> feature_vector_depth;       ///< Depth-based features
    std::vector<float> feature_vector_texture;     ///< Texture features
    std::vector<float> feature_vector_geometric;   ///< Geometric features

    // Advanced ML features
    std::vector<float> deep_learning_features;     ///< Deep learning features
    std::vector<float> multi_scale_features;       ///< Multi-scale features
    std::vector<float> temporal_features;          ///< Temporal consistency features

    // Combined feature vector
    std::vector<float> combined_feature_vector;    ///< Final combined vector

    // Quality assessment
    struct QualityMetrics {
        float overall_quality_score = 0.0f;        ///< Overall quality score
        float feature_completeness = 0.0f;         ///< Feature completeness score
        float feature_consistency = 0.0f;          ///< Feature consistency score
        float extraction_confidence = 0.0f;        ///< Extraction confidence

        // Detailed quality scores
        float image_sharpness = 0.0f;              ///< Image sharpness score
        float illumination_quality = 0.0f;         ///< Illumination quality
        float pose_quality = 0.0f;                 ///< Face pose quality
        float expression_neutrality = 0.0f;        ///< Expression neutrality
        float occlusion_score = 0.0f;              ///< Occlusion assessment

        // Banking-specific quality
        float banking_grade_score = 0.0f;          ///< Banking grade assessment
        bool meets_banking_requirements = false;   ///< Banking requirements met
        std::vector<std::string> quality_issues;   ///< Identified quality issues
    } quality_metrics;

    // Performance metrics
    struct PerformanceMetrics {
        double extraction_time_ms = 0.0;           ///< Total extraction time
        double preprocessing_time_ms = 0.0;        ///< Preprocessing time
        double feature_computation_time_ms = 0.0;  ///< Feature computation time
        double postprocessing_time_ms = 0.0;       ///< Postprocessing time
        size_t memory_usage_bytes = 0;             ///< Memory usage
        int features_extracted = 0;                ///< Number of features extracted
    } performance_metrics;

    // Metadata
    std::string extraction_method;                 ///< Method used for extraction
    std::string feature_version;                   ///< Feature version identifier
    std::chrono::system_clock::time_point timestamp; ///< Extraction timestamp
    std::map<std::string, std::string> metadata;   ///< Additional metadata

    MLFeatureExtractionResult() {
        timestamp = std::chrono::system_clock::now();
        feature_version = "unlook_ml_v2.0";
    }
};

/**
 * @brief Advanced ML Feature Extractor for Banking Applications
 *
 * Professional feature extraction system providing comprehensive multi-modal
 * feature extraction optimized for banking-grade machine learning algorithms.
 *
 * Key Features:
 * - Multi-modal feature extraction (2D + 3D + depth + texture + geometric)
 * - Deep learning-based feature enhancement
 * - Quality-aware feature extraction with banking compliance
 * - Real-time performance with ARM64/CM4/CM5 optimization
 * - Temporal feature consistency for video sequences
 * - Anti-spoofing and liveness-aware feature extraction
 * - Comprehensive quality assessment and defect detection
 * - Thread-safe concurrent processing with SIMD optimization
 *
 * Integration:
 * - Seamless integration with Unlook 3D scanner data pipeline
 * - Compatible with existing facial recognition components
 * - Optimized for Supernova ML algorithm requirements
 * - Banking compliance with comprehensive audit trail
 */
class MLFeatureExtractor {
public:
    /**
     * @brief Constructor with default configuration
     */
    MLFeatureExtractor();

    /**
     * @brief Constructor with custom configuration
     * @param config ML feature extraction configuration
     */
    explicit MLFeatureExtractor(const MLFeatureExtractionConfig& config);

    /**
     * @brief Destructor
     */
    ~MLFeatureExtractor();

    /**
     * @brief Initialize ML feature extractor
     * @param config Feature extraction configuration
     * @return True if initialization successful
     */
    bool initialize(const AdvancedMLConfig& ml_config);

    /**
     * @brief Check if extractor is initialized and ready
     * @return True if ready for feature extraction
     */
    bool isInitialized() const;

    /**
     * @brief Extract comprehensive ML features from biometric template
     * @param template_data Input biometric template
     * @param enhanced_features Output enhanced feature vector
     * @return FaceResultCode indicating extraction result
     */
    FaceResultCode extractEnhancedFeatures(const BiometricTemplate& template_data,
                                          std::vector<float>& enhanced_features);

    /**
     * @brief Extract comprehensive ML features with detailed results
     * @param template_data Input biometric template
     * @param extraction_result Output detailed extraction result
     * @return FaceResultCode indicating extraction result
     */
    FaceResultCode extractMLFeatures(const BiometricTemplate& template_data,
                                    MLFeatureExtractionResult& extraction_result);

    /**
     * @brief Extract multi-modal features from stereo images and depth
     * @param left_image Left stereo image
     * @param right_image Right stereo image
     * @param depth_map Depth map from stereo processing
     * @param landmarks Facial landmarks (68 or 468 points)
     * @param extraction_result Output extraction result
     * @return FaceResultCode indicating extraction result
     */
    FaceResultCode extractMultiModalFeatures(const cv::Mat& left_image,
                                            const cv::Mat& right_image,
                                            const cv::Mat& depth_map,
                                            const FacialLandmarks& landmarks,
                                            MLFeatureExtractionResult& extraction_result);

    /**
     * @brief Banking-grade feature extraction with strict quality requirements
     * @param template_data Input biometric template
     * @param banking_context Banking compliance context
     * @param extraction_result Output banking-grade result
     * @return FaceResultCode indicating extraction result
     */
    FaceResultCode extractBankingGradeFeatures(const BiometricTemplate& template_data,
                                              const std::map<std::string, std::string>& banking_context,
                                              MLFeatureExtractionResult& extraction_result);

    /**
     * @brief Assess template quality using ML-based analysis
     * @param template_data Template to assess
     * @param quality_report Output detailed quality report
     * @return Overall quality score (0-1)
     */
    float assessTemplateQuality(const BiometricTemplate& template_data,
                               std::map<std::string, float>& quality_report);

    /**
     * @brief Extract temporal features from sequence of templates
     * @param template_sequence Vector of templates in temporal order
     * @param temporal_features Output temporal feature vector
     * @return FaceResultCode indicating extraction result
     */
    FaceResultCode extractTemporalFeatures(const std::vector<BiometricTemplate>& template_sequence,
                                          std::vector<float>& temporal_features);

    /**
     * @brief Extract liveness-aware features for anti-spoofing
     * @param rgb_frames Sequence of RGB frames
     * @param depth_frames Corresponding depth frames
     * @param liveness_features Output liveness feature vector
     * @return FaceResultCode indicating extraction result
     */
    FaceResultCode extractLivenessFeatures(const std::vector<cv::Mat>& rgb_frames,
                                          const std::vector<cv::Mat>& depth_frames,
                                          std::vector<float>& liveness_features);

    /**
     * @brief Optimize feature vector for ML algorithm performance
     * @param raw_features Input raw feature vector
     * @param optimization_params Optimization parameters
     * @param optimized_features Output optimized features
     * @return FaceResultCode indicating optimization result
     */
    FaceResultCode optimizeFeaturesForML(const std::vector<float>& raw_features,
                                        const std::map<std::string, float>& optimization_params,
                                        std::vector<float>& optimized_features);

    /**
     * @brief Extract quality-aware features with adaptive enhancement
     * @param template_data Input template
     * @param quality_threshold Minimum quality threshold
     * @param enhanced_features Output quality-enhanced features
     * @return FaceResultCode indicating extraction result
     */
    FaceResultCode extractQualityAwareFeatures(const BiometricTemplate& template_data,
                                              float quality_threshold,
                                              std::vector<float>& enhanced_features);

    /**
     * @brief Batch feature extraction with parallel processing
     * @param templates Vector of input templates
     * @param extraction_results Output extraction results
     * @param progress_callback Optional progress callback (0-100%)
     * @return Number of successfully processed templates
     */
    size_t extractFeaturesBatch(const std::vector<BiometricTemplate>& templates,
                               std::vector<MLFeatureExtractionResult>& extraction_results,
                               std::function<void(float, size_t, size_t)> progress_callback = nullptr);

    /**
     * @brief Set feature extraction configuration
     * @param config New configuration
     * @return True if configuration applied successfully
     */
    bool setConfiguration(const MLFeatureExtractionConfig& config);

    /**
     * @brief Get current feature extraction configuration
     * @return Current configuration
     */
    MLFeatureExtractionConfig getConfiguration() const;

    /**
     * @brief Enable/disable specific feature extraction modes
     * @param enable_2d Enable 2D features
     * @param enable_3d Enable 3D features
     * @param enable_depth Enable depth features
     * @param enable_ml Enable ML-enhanced features
     */
    void setFeatureExtractionModes(bool enable_2d,
                                  bool enable_3d,
                                  bool enable_depth,
                                  bool enable_ml);

    /**
     * @brief Set banking compliance mode for feature extraction
     * @param enable Enable banking compliance
     * @param quality_threshold Banking quality threshold
     * @param require_liveness Require liveness features
     */
    void setBankingComplianceMode(bool enable,
                                 float quality_threshold = 0.85f,
                                 bool require_liveness = true);

    /**
     * @brief Get feature extraction statistics
     * @param total_extractions Total extractions performed
     * @param successful_extractions Successful extractions
     * @param failed_extractions Failed extractions
     * @param avg_extraction_time Average extraction time (ms)
     * @param avg_quality_score Average quality score
     */
    void getExtractionStatistics(size_t& total_extractions,
                                size_t& successful_extractions,
                                size_t& failed_extractions,
                                double& avg_extraction_time,
                                float& avg_quality_score) const;

    /**
     * @brief Reset extraction statistics
     */
    void resetStatistics();

    /**
     * @brief Get last extraction error
     * @return Human-readable error message
     */
    std::string getLastError() const;

    /**
     * @brief Validate feature extraction system integrity
     * @param integrity_report Output integrity validation report
     * @return True if all components are functioning correctly
     */
    bool validateSystemIntegrity(std::map<std::string, std::string>& integrity_report);

private:
    // Configuration and state
    MLFeatureExtractionConfig config_;
    mutable std::mutex config_mutex_;
    std::atomic<bool> initialized_{false};
    std::atomic<bool> banking_compliance_enabled_{false};

    // Processing components
    std::unique_ptr<cv::HOGDescriptor> hog_descriptor_;
    std::unique_ptr<cv::SIFT> sift_detector_;
    std::vector<cv::Mat> feature_templates_;
    std::map<std::string, cv::Mat> pca_matrices_;

    // Performance monitoring
    mutable std::mutex stats_mutex_;
    std::atomic<size_t> total_extractions_{0};
    std::atomic<size_t> successful_extractions_{0};
    std::atomic<size_t> failed_extractions_{0};
    std::atomic<double> total_extraction_time_{0.0};
    std::atomic<float> cumulative_quality_score_{0.0f};

    // Feature caching
    std::mutex cache_mutex_;
    std::map<std::string, std::vector<float>> feature_cache_;
    size_t max_cache_size_ = 1000;

    // Error handling
    mutable std::mutex error_mutex_;
    std::string last_error_;
    std::vector<std::string> error_history_;

    /**
     * @brief Initialize feature extraction components
     * @return True if initialization successful
     */
    bool initializeComponents();

    /**
     * @brief Extract 2D facial features using traditional methods
     * @param image Input face image
     * @param landmarks Facial landmarks
     * @param features_2d Output 2D features
     * @return FaceResultCode indicating extraction result
     */
    FaceResultCode extract2DFeatures(const cv::Mat& image,
                                    const FacialLandmarks& landmarks,
                                    std::vector<float>& features_2d);

    /**
     * @brief Extract 3D landmark-based features
     * @param landmarks_3d 3D facial landmarks
     * @param features_3d Output 3D features
     * @return FaceResultCode indicating extraction result
     */
    FaceResultCode extract3DFeatures(const std::vector<cv::Point3f>& landmarks_3d,
                                    std::vector<float>& features_3d);

    /**
     * @brief Extract depth-based features from depth map
     * @param depth_map Input depth map
     * @param face_region Face region in depth map
     * @param depth_features Output depth features
     * @return FaceResultCode indicating extraction result
     */
    FaceResultCode extractDepthFeatures(const cv::Mat& depth_map,
                                       const cv::Rect& face_region,
                                       std::vector<float>& depth_features);

    /**
     * @brief Extract texture features using advanced descriptors
     * @param image Input face image
     * @param face_region Face region
     * @param texture_features Output texture features
     * @return FaceResultCode indicating extraction result
     */
    FaceResultCode extractTextureFeatures(const cv::Mat& image,
                                         const cv::Rect& face_region,
                                         std::vector<float>& texture_features);

    /**
     * @brief Extract geometric features from landmarks
     * @param landmarks Facial landmarks
     * @param geometric_features Output geometric features
     * @return FaceResultCode indicating extraction result
     */
    FaceResultCode extractGeometricFeatures(const FacialLandmarks& landmarks,
                                           std::vector<float>& geometric_features);

    /**
     * @brief Enhance features using deep learning methods
     * @param input_features Input feature vector
     * @param enhanced_features Output enhanced features
     * @return FaceResultCode indicating enhancement result
     */
    FaceResultCode enhanceWithDeepLearning(const std::vector<float>& input_features,
                                          std::vector<float>& enhanced_features);

    /**
     * @brief Combine multiple feature vectors into single vector
     * @param feature_vectors Vector of feature vectors to combine
     * @param combination_method Combination method ("concatenate", "weighted", "pca")
     * @param combined_features Output combined features
     * @return FaceResultCode indicating combination result
     */
    FaceResultCode combineFeatureVectors(const std::vector<std::vector<float>>& feature_vectors,
                                        const std::string& combination_method,
                                        std::vector<float>& combined_features);

    /**
     * @brief Normalize feature vector using specified method
     * @param features Input/output feature vector
     * @param method Normalization method ("L1", "L2", "MinMax")
     * @return FaceResultCode indicating normalization result
     */
    FaceResultCode normalizeFeatures(std::vector<float>& features,
                                    const std::string& method);

    /**
     * @brief Assess feature quality using multiple metrics
     * @param features Feature vector to assess
     * @param template_data Original template data
     * @param quality_metrics Output quality metrics
     * @return Overall quality score
     */
    float assessFeatureQuality(const std::vector<float>& features,
                              const BiometricTemplate& template_data,
                              MLFeatureExtractionResult::QualityMetrics& quality_metrics);

    /**
     * @brief Detect feature extraction defects
     * @param features Feature vector
     * @param extraction_result Extraction result context
     * @param defects Output list of detected defects
     * @return Number of defects detected
     */
    size_t detectExtractionDefects(const std::vector<float>& features,
                                  const MLFeatureExtractionResult& extraction_result,
                                  std::vector<std::string>& defects);

    /**
     * @brief Update extraction statistics
     * @param extraction_time Processing time
     * @param success Whether extraction was successful
     * @param quality_score Quality score if available
     */
    void updateStatistics(double extraction_time, bool success, float quality_score = -1.0f);

    /**
     * @brief Generate cache key for feature caching
     * @param template_data Template data
     * @param config_hash Configuration hash
     * @return Cache key string
     */
    std::string generateCacheKey(const BiometricTemplate& template_data,
                                const std::string& config_hash);

    /**
     * @brief Set last error message
     * @param error_message Error message to set
     */
    void setLastError(const std::string& error_message);

    // Disable copy constructor and assignment
    MLFeatureExtractor(const MLFeatureExtractor&) = delete;
    MLFeatureExtractor& operator=(const MLFeatureExtractor&) = delete;

    // Enable move semantics
    MLFeatureExtractor(MLFeatureExtractor&&) noexcept;
    MLFeatureExtractor& operator=(MLFeatureExtractor&&) noexcept;
};

/**
 * @brief ML Feature Extractor utility functions
 */
class MLFeatureExtractorUtils {
public:
    /**
     * @brief Validate ML feature extraction configuration
     * @param config Configuration to validate
     * @param validation_errors Output validation errors
     * @return True if configuration is valid
     */
    static bool validateConfiguration(const MLFeatureExtractionConfig& config,
                                     std::vector<std::string>& validation_errors);

    /**
     * @brief Generate feature extraction performance report
     * @param results Vector of extraction results
     * @param include_recommendations Include performance recommendations
     * @return Formatted performance report
     */
    static std::string generatePerformanceReport(
        const std::vector<MLFeatureExtractionResult>& results,
        bool include_recommendations = true);

    /**
     * @brief Estimate resource requirements for feature extraction
     * @param config Feature extraction configuration
     * @param expected_load Expected processing load
     * @return Resource requirement estimates
     */
    static std::map<std::string, std::string> estimateResourceRequirements(
        const MLFeatureExtractionConfig& config,
        const std::map<std::string, float>& expected_load);

    /**
     * @brief Compare feature vectors for similarity analysis
     * @param features1 First feature vector
     * @param features2 Second feature vector
     * @param similarity_method Method for similarity computation
     * @return Similarity score (0-1)
     */
    static float compareFeatureVectors(const std::vector<float>& features1,
                                      const std::vector<float>& features2,
                                      const std::string& similarity_method = "cosine");

    /**
     * @brief Validate feature vector integrity
     * @param features Feature vector to validate
     * @param expected_size Expected feature vector size
     * @param validation_report Output validation report
     * @return True if feature vector is valid
     */
    static bool validateFeatureVector(const std::vector<float>& features,
                                     size_t expected_size,
                                     std::map<std::string, std::string>& validation_report);
};

} // namespace face
} // namespace unlook