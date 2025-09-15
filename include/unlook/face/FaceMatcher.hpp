#pragma once

#include "unlook/face/FaceTypes.hpp"
#include <opencv2/opencv.hpp>
#include <memory>
#include <vector>
#include <string>
#include <mutex>
#include <atomic>
#include <unordered_map>

namespace unlook {
namespace face {

/**
 * @brief Face matching configuration for authentication
 */
struct FaceMatchingConfig {
    // Matching method
    enum Method {
        COSINE_SIMILARITY,      ///< Cosine similarity matching
        EUCLIDEAN_DISTANCE,     ///< Euclidean distance matching
        MAHALANOBIS_DISTANCE,   ///< Mahalanobis distance matching
        NEURAL_SIMILARITY,      ///< Neural network similarity
        BANKING_GRADE_HYBRID    ///< Banking-grade hybrid matching
    } method = BANKING_GRADE_HYBRID;

    // Authentication mode
    AuthenticationMode auth_mode = AuthenticationMode::VERIFY_1_TO_1;

    // Threshold settings
    float verification_threshold = 0.7f;       ///< 1:1 verification threshold
    float identification_threshold = 0.8f;     ///< 1:N identification threshold
    float banking_threshold = 0.95f;          ///< Banking-grade threshold
    bool adaptive_threshold = true;            ///< Enable adaptive thresholding

    // Quality gates
    float min_template_quality = 0.8f;        ///< Minimum template quality
    float min_probe_quality = 0.8f;           ///< Minimum probe quality
    bool enable_quality_gating = true;        ///< Enable quality-based gating

    // Banking compliance
    float target_far = 0.0001f;               ///< Target false accept rate (0.01%)
    float target_frr = 0.03f;                 ///< Target false reject rate (3%)
    bool iso_compliance_mode = false;          ///< ISO/IEC 19794-5 compliance
    bool banking_audit_mode = false;          ///< Enable banking audit trail

    // Security features
    bool enable_template_encryption = true;   ///< Require encrypted templates
    bool enable_anti_spoofing = true;         ///< Enable anti-spoofing checks
    bool enable_velocity_checking = false;    ///< Enable velocity checking
    int max_attempts_per_minute = 5;          ///< Maximum authentication attempts

    // Performance settings
    bool enable_template_caching = true;      ///< Cache decrypted templates
    int cache_size_mb = 64;                   ///< Template cache size
    bool enable_parallel_matching = true;     ///< Parallel 1:N matching
    int max_matching_threads = 4;             ///< Maximum matching threads

    // Database settings
    int max_database_size = 100000;           ///< Maximum template database size
    bool enable_database_indexing = true;     ///< Enable fast database indexing
    float index_threshold = 0.5f;             ///< Indexing threshold

    bool validate() const;
    std::string toString() const;
};

/**
 * @brief Banking-grade facial recognition matcher
 *
 * High-security facial recognition matching system designed for banking
 * applications with strict FAR/FRR requirements. Provides 1:1 verification
 * and 1:N identification with comprehensive security features.
 *
 * Key features:
 * - Banking-grade matching with FAR < 0.001%, FRR < 3%
 * - Multiple similarity metrics with adaptive thresholding
 * - Encrypted template handling with secure decryption
 * - Comprehensive audit trail and compliance logging
 * - Anti-spoofing and velocity checking integration
 * - High-performance database operations with indexing
 * - Thread-safe concurrent matching operations
 * - ARM64/NEON optimized for CM4/CM5 performance
 *
 * Integrates seamlessly with Supernova ML for enhanced accuracy.
 */
class FaceMatcher {
public:
    /**
     * @brief Constructor with default configuration
     */
    FaceMatcher();

    /**
     * @brief Constructor with custom configuration
     * @param config Matching configuration
     */
    explicit FaceMatcher(const FaceMatchingConfig& config);

    /**
     * @brief Destructor
     */
    ~FaceMatcher();

    /**
     * @brief Initialize matcher with encryption keys
     * @param encryption_key_path Path to encryption key file
     * @param database_path Path to template database
     * @return FaceResultCode indicating success or failure
     */
    FaceResultCode initialize(const std::string& encryption_key_path,
                             const std::string& database_path = "");

    /**
     * @brief Check if matcher is initialized
     * @return True if matcher is ready for use
     */
    bool isInitialized() const;

    /**
     * @brief Perform 1:1 verification
     * @param probe_template Probe template to verify
     * @param reference_template Reference template to match against
     * @param match_result Output matching result
     * @return FaceResultCode indicating success or failure
     */
    FaceResultCode verify(const BiometricTemplate& probe_template,
                         const BiometricTemplate& reference_template,
                         FaceMatchResult& match_result);

    /**
     * @brief Perform 1:N identification
     * @param probe_template Probe template to identify
     * @param database_templates Vector of database templates
     * @param match_results Output vector of match results (sorted by similarity)
     * @param max_candidates Maximum number of candidates to return
     * @return FaceResultCode indicating success or failure
     */
    FaceResultCode identify(const BiometricTemplate& probe_template,
                           const std::vector<BiometricTemplate>& database_templates,
                           std::vector<FaceMatchResult>& match_results,
                           int max_candidates = 5);

    /**
     * @brief Perform banking-grade verification with enhanced security
     * @param probe_template Probe template
     * @param reference_template Reference template
     * @param liveness_result Liveness detection result
     * @param match_result Output matching result with security metrics
     * @return FaceResultCode indicating success or failure
     */
    FaceResultCode verifyBankingGrade(const BiometricTemplate& probe_template,
                                     const BiometricTemplate& reference_template,
                                     const LivenessResult& liveness_result,
                                     FaceMatchResult& match_result);

    /**
     * @brief Asynchronous verification
     * @param probe_template Probe template
     * @param reference_template Reference template
     * @param callback Callback function for result
     * @return True if matching task was queued successfully
     */
    bool verifyAsync(const BiometricTemplate& probe_template,
                    const BiometricTemplate& reference_template,
                    FaceMatchingCallback callback);

    /**
     * @brief Batch verification for multiple probes
     * @param probe_templates Vector of probe templates
     * @param reference_templates Vector of reference templates
     * @param match_results Output vector of matching results
     * @param progress_callback Optional progress callback (0-100)
     * @return Number of successfully processed matches
     */
    size_t verifyBatch(const std::vector<BiometricTemplate>& probe_templates,
                      const std::vector<BiometricTemplate>& reference_templates,
                      std::vector<FaceMatchResult>& match_results,
                      std::function<void(float)> progress_callback = nullptr);

    /**
     * @brief Load template database from file
     * @param database_path Path to template database
     * @param encryption_key Decryption key for encrypted database
     * @return FaceResultCode indicating success or failure
     */
    FaceResultCode loadDatabase(const std::string& database_path,
                               const std::array<uint8_t, 32>& encryption_key);

    /**
     * @brief Add template to in-memory database
     * @param template_data Template to add
     * @param user_id User identifier for template
     * @return FaceResultCode indicating success or failure
     */
    FaceResultCode addToDatabase(const BiometricTemplate& template_data,
                                const std::string& user_id);

    /**
     * @brief Remove template from database
     * @param template_id Template identifier to remove
     * @return FaceResultCode indicating success or failure
     */
    FaceResultCode removeFromDatabase(const std::string& template_id);

    /**
     * @brief Update existing template in database
     * @param template_id Template identifier to update
     * @param new_template New template data
     * @return FaceResultCode indicating success or failure
     */
    FaceResultCode updateInDatabase(const std::string& template_id,
                                   const BiometricTemplate& new_template);

    /**
     * @brief Search database for similar templates
     * @param query_template Query template
     * @param similarity_threshold Minimum similarity threshold
     * @param max_results Maximum number of results
     * @param search_results Output search results
     * @return FaceResultCode indicating success or failure
     */
    FaceResultCode searchDatabase(const BiometricTemplate& query_template,
                                 float similarity_threshold,
                                 int max_results,
                                 std::vector<FaceMatchResult>& search_results);

    /**
     * @brief Optimize database for faster searching
     * @param index_method Indexing method ("lsh", "kd_tree", "ann")
     * @param index_parameters Indexing parameters
     * @return FaceResultCode indicating success or failure
     */
    FaceResultCode optimizeDatabase(const std::string& index_method,
                                   const std::map<std::string, float>& index_parameters);

    /**
     * @brief Set matching configuration
     * @param config New configuration
     * @return FaceResultCode indicating success or failure
     */
    FaceResultCode setConfiguration(const FaceMatchingConfig& config);

    /**
     * @brief Get current configuration
     * @return Current matching configuration
     */
    FaceMatchingConfig getConfiguration() const;

    /**
     * @brief Set encryption key for template decryption
     * @param encryption_key 256-bit encryption key
     * @return FaceResultCode indicating success or failure
     */
    FaceResultCode setEncryptionKey(const std::array<uint8_t, 32>& encryption_key);

    /**
     * @brief Calibrate matching thresholds using validation data
     * @param validation_templates Validation template pairs
     * @param genuine_pairs Indices of genuine pairs
     * @param impostor_pairs Indices of impostor pairs
     * @param target_far Target false accept rate
     * @param optimized_threshold Output optimized threshold
     * @return FaceResultCode indicating success or failure
     */
    FaceResultCode calibrateThresholds(
        const std::vector<std::pair<BiometricTemplate, BiometricTemplate>>& validation_templates,
        const std::vector<int>& genuine_pairs,
        const std::vector<int>& impostor_pairs,
        float target_far,
        float& optimized_threshold);

    /**
     * @brief Compute matching performance metrics
     * @param test_results Vector of test matching results
     * @param ground_truth Ground truth labels (true/false for genuine/impostor)
     * @param threshold Decision threshold
     * @param far Output false accept rate
     * @param frr Output false reject rate
     * @param eer Output equal error rate
     * @return FaceResultCode indicating success or failure
     */
    FaceResultCode computePerformanceMetrics(
        const std::vector<FaceMatchResult>& test_results,
        const std::vector<bool>& ground_truth,
        float threshold,
        float& far,
        float& frr,
        float& eer);

    /**
     * @brief Enable/disable specific matching features
     * @param enable_quality_gating Enable quality-based gating
     * @param enable_anti_spoofing Enable anti-spoofing checks
     * @param enable_audit_logging Enable audit trail logging
     */
    void setFeatures(bool enable_quality_gating,
                    bool enable_anti_spoofing,
                    bool enable_audit_logging);

    /**
     * @brief Get matching statistics
     * @param avg_matching_time Average matching time in milliseconds
     * @param total_matches Total number of matches performed
     * @param genuine_matches Number of genuine matches
     * @param impostor_matches Number of impostor matches
     * @param current_far Current false accept rate
     * @param current_frr Current false reject rate
     */
    void getStatistics(double& avg_matching_time,
                      size_t& total_matches,
                      size_t& genuine_matches,
                      size_t& impostor_matches,
                      float& current_far,
                      float& current_frr) const;

    /**
     * @brief Reset matching statistics
     */
    void resetStatistics();

    /**
     * @brief Cancel ongoing asynchronous operations
     */
    void cancelAsync();

    /**
     * @brief Get last error message
     * @return Human-readable error message
     */
    std::string getLastError() const;

    /**
     * @brief Get database statistics
     * @param num_templates Number of templates in database
     * @param total_size_mb Total database size in MB
     * @param index_status Indexing status
     */
    void getDatabaseStatistics(size_t& num_templates,
                              float& total_size_mb,
                              std::string& index_status) const;

    /**
     * @brief Export matching audit log
     * @param output_path Path to export audit log
     * @param format Export format ("json", "csv", "xml")
     * @return FaceResultCode indicating success or failure
     */
    FaceResultCode exportAuditLog(const std::string& output_path,
                                 const std::string& format = "json");

    /**
     * @brief Validate template compatibility
     * @param template_data Template to validate
     * @param compatibility_info Output compatibility information
     * @return True if template is compatible
     */
    bool validateTemplateCompatibility(const BiometricTemplate& template_data,
                                      std::string& compatibility_info);

    /**
     * @brief Enable banking-grade matching mode
     * @param enable Enable/disable banking mode
     * @param strict_security Enable strict security measures
     * @param audit_compliance Enable full audit compliance
     */
    void setBankingMode(bool enable,
                       bool strict_security = true,
                       bool audit_compliance = true);

private:
    class Impl;
    std::unique_ptr<Impl> pImpl;

    // Configuration and state
    FaceMatchingConfig config_;
    std::array<uint8_t, 32> encryption_key_;
    bool has_encryption_key_{false};

    // Template database
    std::mutex database_mutex_;
    std::unordered_map<std::string, BiometricTemplate> template_database_;
    std::unordered_map<std::string, std::string> template_to_user_map_;

    // Performance monitoring
    mutable std::mutex stats_mutex_;
    std::atomic<size_t> total_matches_{0};
    std::atomic<size_t> genuine_matches_{0};
    std::atomic<size_t> impostor_matches_{0};
    std::atomic<double> total_matching_time_{0.0};
    std::atomic<size_t> correct_genuine_{0};
    std::atomic<size_t> correct_impostor_{0};

    // Template cache for performance
    std::mutex cache_mutex_;
    std::unordered_map<std::string, std::vector<float>> decrypted_template_cache_;
    size_t cache_hit_count_{0};
    size_t cache_miss_count_{0};

    /**
     * @brief Internal matching implementation
     * @param probe_features Probe feature vector
     * @param reference_features Reference feature vector
     * @param similarity_score Output similarity score
     * @param confidence_level Output confidence level
     * @return FaceResultCode
     */
    FaceResultCode matchInternal(const std::vector<float>& probe_features,
                                const std::vector<float>& reference_features,
                                float& similarity_score,
                                float& confidence_level);

    /**
     * @brief Decrypt and cache template features
     * @param template_data Input encrypted template
     * @param features Output decrypted features
     * @return FaceResultCode
     */
    FaceResultCode getTemplateFeatures(const BiometricTemplate& template_data,
                                      std::vector<float>& features);

    /**
     * @brief Compute cosine similarity between feature vectors
     * @param features1 First feature vector
     * @param features2 Second feature vector
     * @return Cosine similarity (0-1)
     */
    float computeCosineSimilarity(const std::vector<float>& features1,
                                 const std::vector<float>& features2);

    /**
     * @brief Compute Euclidean distance between feature vectors
     * @param features1 First feature vector
     * @param features2 Second feature vector
     * @return Euclidean distance
     */
    float computeEuclideanDistance(const std::vector<float>& features1,
                                  const std::vector<float>& features2);

    /**
     * @brief Compute Mahalanobis distance with covariance matrix
     * @param features1 First feature vector
     * @param features2 Second feature vector
     * @param covariance_matrix Covariance matrix
     * @return Mahalanobis distance
     */
    float computeMahalanobisDistance(const std::vector<float>& features1,
                                    const std::vector<float>& features2,
                                    const cv::Mat& covariance_matrix);

    /**
     * @brief Apply adaptive thresholding based on quality scores
     * @param base_threshold Base threshold value
     * @param probe_quality Probe template quality
     * @param reference_quality Reference template quality
     * @return Adjusted threshold
     */
    float adaptThreshold(float base_threshold,
                        float probe_quality,
                        float reference_quality);

    /**
     * @brief Validate template quality for matching
     * @param template_data Input template
     * @param quality_score Output quality score
     * @return True if template meets quality requirements
     */
    bool validateTemplateQuality(const BiometricTemplate& template_data,
                                float& quality_score);

    /**
     * @brief Log matching attempt for audit trail
     * @param probe_id Probe template ID
     * @param reference_id Reference template ID
     * @param match_result Matching result
     * @param timestamp Matching timestamp
     */
    void logMatchingAttempt(const std::string& probe_id,
                           const std::string& reference_id,
                           const FaceMatchResult& match_result,
                           const std::chrono::system_clock::time_point& timestamp);

    /**
     * @brief Update performance statistics
     * @param matching_time Time taken for matching
     * @param is_genuine Whether match was genuine
     * @param is_correct Whether decision was correct
     * @param similarity_score Similarity score achieved
     */
    void updateStatistics(double matching_time,
                         bool is_genuine,
                         bool is_correct,
                         float similarity_score);

    /**
     * @brief Manage template cache
     * @param template_id Template identifier
     * @param features Feature vector to cache
     */
    void updateTemplateCache(const std::string& template_id,
                            const std::vector<float>& features);

    /**
     * @brief Check velocity limits for user
     * @param user_id User identifier
     * @return True if within velocity limits
     */
    bool checkVelocityLimits(const std::string& user_id);

    // Disable copy constructor and assignment
    FaceMatcher(const FaceMatcher&) = delete;
    FaceMatcher& operator=(const FaceMatcher&) = delete;

    // Enable move semantics
    FaceMatcher(FaceMatcher&&) noexcept;
    FaceMatcher& operator=(FaceMatcher&&) noexcept;
};

/**
 * @brief Matching performance evaluation utilities
 */
class MatchingEvaluator {
public:
    /**
     * @brief Compute ROC curve points
     * @param similarity_scores Vector of similarity scores
     * @param ground_truth Ground truth labels
     * @param roc_points Output ROC curve points (FAR, FRR)
     */
    static void computeROCCurve(const std::vector<float>& similarity_scores,
                               const std::vector<bool>& ground_truth,
                               std::vector<std::pair<float, float>>& roc_points);

    /**
     * @brief Find optimal threshold for target FAR
     * @param similarity_scores Vector of similarity scores
     * @param ground_truth Ground truth labels
     * @param target_far Target false accept rate
     * @param optimal_threshold Output optimal threshold
     * @param achieved_frr Output achieved false reject rate
     * @return True if optimal threshold found
     */
    static bool findOptimalThreshold(const std::vector<float>& similarity_scores,
                                    const std::vector<bool>& ground_truth,
                                    float target_far,
                                    float& optimal_threshold,
                                    float& achieved_frr);

    /**
     * @brief Compute equal error rate
     * @param similarity_scores Vector of similarity scores
     * @param ground_truth Ground truth labels
     * @param eer_threshold Output EER threshold
     * @return Equal error rate value
     */
    static float computeEER(const std::vector<float>& similarity_scores,
                           const std::vector<bool>& ground_truth,
                           float& eer_threshold);

    /**
     * @brief Generate matching performance report
     * @param test_results Vector of test results
     * @param ground_truth Ground truth labels
     * @param threshold Decision threshold
     * @return Performance report as formatted string
     */
    static std::string generatePerformanceReport(
        const std::vector<FaceMatchResult>& test_results,
        const std::vector<bool>& ground_truth,
        float threshold);
};

} // namespace face
} // namespace unlook