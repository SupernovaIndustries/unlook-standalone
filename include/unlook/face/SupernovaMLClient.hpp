#pragma once

/**
 * @file SupernovaMLClient.hpp
 * @brief Enhanced ML Client for Supernova Banking Algorithms Integration
 *
 * Professional ML client implementation providing comprehensive banking-grade
 * machine learning integration with advanced security, performance monitoring,
 * and compliance features for the Unlook 3D Scanner facial recognition system.
 *
 * @version 2.0.0-banking-enhanced
 * @author Unlook Team - API Architecture Agent
 * @copyright 2025 Unlook. All rights reserved.
 */

#include "unlook/face/SupernovaMLInterface.hpp"
#include "unlook/face/FaceTypes.hpp"
#include "unlook/core/types.hpp"
#include "unlook/core/Logger.hpp"

#include <opencv2/opencv.hpp>
#include <memory>
#include <vector>
#include <string>
#include <map>
#include <functional>
#include <mutex>
#include <atomic>
#include <future>
#include <chrono>
#include <queue>
#include <thread>
#include <condition_variable>

namespace unlook {
namespace face {

// Forward declarations for enhanced components
class MLFeatureExtractor;
class SecureTelemetryLogger;
class MLTemplateManager;
class SupernovaAPIConnector;
class MLPerformanceMonitor;
class BankingMLValidator;

/**
 * @brief Advanced ML processing configuration for banking applications
 */
struct AdvancedMLConfig {
    // Enhanced processing settings
    bool enable_deep_learning_features = true;     ///< Enable advanced DL features
    bool enable_multi_modal_fusion = true;         ///< Enable 2D+3D+depth fusion
    bool enable_temporal_stability = true;         ///< Enable temporal consistency
    bool enable_quality_enhancement = true;        ///< Enable quality preprocessing

    // ML model settings
    std::string primary_model_version = "supernova-v3.5";
    std::string fallback_model_version = "supernova-v3.0";
    float model_confidence_threshold = 0.95f;      ///< Minimum confidence for banking
    float quality_threshold = 0.85f;               ///< Minimum quality threshold

    // Banking-specific settings
    float banking_far_requirement = 0.0001f;       ///< 0.01% FAR requirement
    float banking_frr_requirement = 0.03f;         ///< 3% FRR requirement
    bool require_liveness_validation = true;       ///< Require liveness for banking
    bool enable_presentation_attack_detection = true; ///< Enable PAD

    // Performance settings
    int max_concurrent_requests = 10;              ///< Maximum concurrent ML requests
    int request_timeout_ms = 10000;                ///< 10 second timeout for banking
    int retry_attempts = 3;                        ///< Retry attempts for failed requests
    int exponential_backoff_base_ms = 1000;        ///< Base backoff time

    // Security settings
    bool require_end_to_end_encryption = true;     ///< E2E encryption requirement
    std::string encryption_algorithm = "AES-256-GCM"; ///< Encryption algorithm
    bool enable_secure_enclave = true;             ///< Use secure enclave if available
    bool require_certificate_pinning = true;       ///< Certificate pinning

    // Compliance settings
    std::string compliance_level = "PCI_DSS_L1";   ///< Compliance level
    bool enable_gdpr_mode = true;                  ///< GDPR compliance mode
    bool require_audit_trail = true;               ///< Full audit trail
    bool enable_data_residency = true;             ///< Data residency controls

    bool validate() const;
    std::string toString() const;
};

/**
 * @brief Advanced ML processing result with comprehensive metrics
 */
struct AdvancedMLResult {
    // Core ML results
    SupernovaVerificationResult verification_result;

    // Enhanced metrics
    struct QualityAssessment {
        float image_quality_score = 0.0f;          ///< Input image quality
        float feature_quality_score = 0.0f;        ///< Feature extraction quality
        float template_quality_score = 0.0f;       ///< Template quality
        float overall_quality_score = 0.0f;        ///< Overall quality assessment
        std::vector<std::string> quality_issues;   ///< Quality issues detected
    } quality_assessment;

    struct PerformanceMetrics {
        double feature_extraction_time_ms = 0.0;   ///< Feature extraction time
        double ml_inference_time_ms = 0.0;         ///< ML model inference time
        double total_processing_time_ms = 0.0;     ///< Total processing time
        double network_latency_ms = 0.0;           ///< Network round-trip time
        size_t data_transferred_bytes = 0;         ///< Total data transferred
    } performance_metrics;

    struct SecurityMetrics {
        bool encryption_validated = false;         ///< Encryption validation status
        bool certificate_validated = false;       ///< Certificate validation status
        bool data_integrity_verified = false;     ///< Data integrity verification
        std::string security_level_achieved;      ///< Achieved security level
        std::vector<std::string> security_warnings; ///< Security warnings
    } security_metrics;

    struct ComplianceMetrics {
        bool gdpr_compliant = false;               ///< GDPR compliance status
        bool banking_compliant = false;           ///< Banking compliance status
        std::string audit_trail_id;               ///< Audit trail identifier
        std::map<std::string, std::string> compliance_metadata; ///< Compliance metadata
    } compliance_metrics;

    AdvancedMLResult() = default;
};

/**
 * @brief ML processing request with enhanced context
 */
struct MLProcessingRequest {
    // Core request data
    BiometricTemplate probe_template;
    std::string reference_template_id;
    std::string user_id;
    std::string session_id;

    // Banking transaction context
    struct TransactionContext {
        std::string transaction_id;
        std::string transaction_type;
        float transaction_amount = 0.0f;
        std::string merchant_id;
        std::string terminal_id;
        std::chrono::system_clock::time_point timestamp;
    } transaction_context;

    // Processing options
    struct ProcessingOptions {
        bool require_liveness_check = true;
        bool enable_presentation_attack_detection = true;
        bool require_banking_grade_match = true;
        float custom_threshold = -1.0f;            ///< Use custom threshold if > 0
        std::string processing_priority = "HIGH";   ///< HIGH, MEDIUM, LOW
    } processing_options;

    // Quality requirements
    struct QualityRequirements {
        float minimum_image_quality = 0.7f;
        float minimum_feature_quality = 0.8f;
        float minimum_template_quality = 0.85f;
        bool reject_low_quality = true;
    } quality_requirements;

    MLProcessingRequest() {
        transaction_context.timestamp = std::chrono::system_clock::now();
    }
};

/**
 * @brief Enhanced Supernova ML Client for Banking Applications
 *
 * Professional ML client providing comprehensive machine learning integration
 * with banking-grade security, performance monitoring, and compliance features.
 *
 * Key Features:
 * - Advanced ML feature extraction with multi-modal fusion
 * - Banking-grade security with E2E encryption and secure enclaves
 * - Real-time performance monitoring with SLA compliance
 * - Comprehensive audit trail for regulatory compliance
 * - Adaptive quality enhancement and preprocessing
 * - Failover and redundancy for high availability
 * - Thread-safe concurrent request processing
 * - ARM64/CM4/CM5 optimized for industrial deployment
 *
 * Integration:
 * - Extends existing SupernovaMLInterface
 * - Seamless integration with Unlook face recognition pipeline
 * - Compatible with existing banking compliance validators
 * - Supports both real-time and batch processing modes
 */
class SupernovaMLClient {
public:
    /**
     * @brief Constructor with default configuration
     */
    SupernovaMLClient();

    /**
     * @brief Constructor with advanced configuration
     * @param config Advanced ML configuration
     */
    explicit SupernovaMLClient(const AdvancedMLConfig& config);

    /**
     * @brief Destructor
     */
    ~SupernovaMLClient();

    /**
     * @brief Initialize enhanced ML client with all components
     * @param config Advanced ML configuration
     * @param base_interface Existing Supernova ML interface
     * @return FaceResultCode indicating initialization result
     */
    FaceResultCode initialize(const AdvancedMLConfig& config,
                             std::shared_ptr<SupernovaMLInterface> base_interface);

    /**
     * @brief Check if client is fully initialized and ready
     * @return True if all components are ready for ML processing
     */
    bool isInitialized() const;

    /**
     * @brief Perform enhanced ML verification with comprehensive analysis
     * @param request ML processing request with context
     * @param result Output advanced ML result with metrics
     * @return FaceResultCode indicating processing result
     */
    FaceResultCode processMLVerification(const MLProcessingRequest& request,
                                       AdvancedMLResult& result);

    /**
     * @brief Perform banking-grade ML verification with strict compliance
     * @param request ML processing request
     * @param banking_context Additional banking compliance context
     * @param result Output banking-grade result
     * @return FaceResultCode indicating verification result
     */
    FaceResultCode processBankingGradeVerification(
        const MLProcessingRequest& request,
        const std::map<std::string, std::string>& banking_context,
        AdvancedMLResult& result);

    /**
     * @brief Asynchronous ML processing with callback
     * @param request ML processing request
     * @param callback Result callback function
     * @param priority Processing priority (HIGH, MEDIUM, LOW)
     * @return Future for async operation tracking
     */
    std::future<AdvancedMLResult> processMLVerificationAsync(
        const MLProcessingRequest& request,
        std::function<void(const AdvancedMLResult&)> callback = nullptr,
        const std::string& priority = "MEDIUM");

    /**
     * @brief Batch ML processing with progress tracking
     * @param requests Vector of ML processing requests
     * @param results Output vector of ML results
     * @param progress_callback Progress callback (0-100%)
     * @param max_concurrent Maximum concurrent requests
     * @return Number of successfully processed requests
     */
    size_t processBatchMLVerification(
        const std::vector<MLProcessingRequest>& requests,
        std::vector<AdvancedMLResult>& results,
        std::function<void(float, size_t, size_t)> progress_callback = nullptr,
        int max_concurrent = -1);

    /**
     * @brief Enhanced template enrollment with ML optimization
     * @param enrollment_data Multi-frame enrollment data
     * @param user_context User enrollment context
     * @param optimized_template Output ML-optimized template
     * @return FaceResultCode indicating enrollment result
     */
    FaceResultCode enrollTemplateMLOptimized(
        const std::vector<BiometricTemplate>& enrollment_data,
        const std::map<std::string, std::string>& user_context,
        BiometricTemplate& optimized_template);

    /**
     * @brief ML-enhanced template quality assessment
     * @param template_data Template to assess
     * @param quality_report Output detailed quality report
     * @return Quality score (0-1) with detailed metrics
     */
    float assessTemplateQualityML(const BiometricTemplate& template_data,
                                 std::map<std::string, float>& quality_report);

    /**
     * @brief Adaptive threshold optimization using ML feedback
     * @param historical_results Previous verification results
     * @param target_far Target false accept rate
     * @param target_frr Target false reject rate
     * @return Optimized threshold value
     */
    float optimizeThresholdML(const std::vector<AdvancedMLResult>& historical_results,
                             float target_far = 0.0001f,
                             float target_frr = 0.03f);

    /**
     * @brief Real-time ML model performance monitoring
     * @param performance_metrics Output current performance metrics
     * @return FaceResultCode indicating monitoring result
     */
    FaceResultCode getMLPerformanceMetrics(std::map<std::string, float>& performance_metrics);

    /**
     * @brief ML system health diagnostics
     * @param diagnostic_results Output diagnostic information
     * @return FaceResultCode indicating diagnostic result
     */
    FaceResultCode runMLSystemDiagnostics(std::map<std::string, std::string>& diagnostic_results);

    /**
     * @brief Export comprehensive ML audit trail
     * @param output_path Path to export audit data
     * @param format Export format ("json", "csv", "xml")
     * @param include_templates Include template metadata
     * @param date_range Optional date range filter
     * @return FaceResultCode indicating export result
     */
    FaceResultCode exportMLAuditTrail(const std::string& output_path,
                                     const std::string& format = "json",
                                     bool include_templates = false,
                                     const std::pair<std::string, std::string>& date_range = {});

    /**
     * @brief Configure ML client settings
     * @param config New advanced ML configuration
     * @return FaceResultCode indicating configuration result
     */
    FaceResultCode setConfiguration(const AdvancedMLConfig& config);

    /**
     * @brief Get current ML client configuration
     * @return Current advanced ML configuration
     */
    AdvancedMLConfig getConfiguration() const;

    /**
     * @brief Enable/disable specific ML features
     * @param enable_deep_learning Enable deep learning features
     * @param enable_multi_modal Enable multi-modal processing
     * @param enable_temporal_stability Enable temporal consistency
     * @param enable_quality_enhancement Enable quality preprocessing
     */
    void setMLFeatures(bool enable_deep_learning,
                      bool enable_multi_modal,
                      bool enable_temporal_stability,
                      bool enable_quality_enhancement);

    /**
     * @brief Set banking compliance mode
     * @param enable Enable banking compliance
     * @param compliance_level Compliance level required
     * @param audit_level Audit logging level
     */
    void setBankingComplianceMode(bool enable,
                                 const std::string& compliance_level = "PCI_DSS_L1",
                                 const std::string& audit_level = "COMPREHENSIVE");

    /**
     * @brief Get ML processing statistics
     * @param total_requests Total ML requests processed
     * @param successful_requests Successful ML requests
     * @param failed_requests Failed ML requests
     * @param avg_processing_time Average processing time (ms)
     * @param avg_ml_accuracy Average ML accuracy
     */
    void getMLStatistics(size_t& total_requests,
                        size_t& successful_requests,
                        size_t& failed_requests,
                        double& avg_processing_time,
                        float& avg_ml_accuracy) const;

    /**
     * @brief Reset ML processing statistics
     */
    void resetMLStatistics();

    /**
     * @brief Cancel all pending asynchronous operations
     */
    void cancelAllPendingOperations();

    /**
     * @brief Get last ML processing error
     * @return Human-readable error message
     */
    std::string getLastMLError() const;

    /**
     * @brief Validate ML system integrity
     * @param integrity_report Output integrity validation report
     * @return True if all ML components are functioning correctly
     */
    bool validateMLSystemIntegrity(std::map<std::string, std::string>& integrity_report);

private:
    // Enhanced ML components
    std::unique_ptr<MLFeatureExtractor> feature_extractor_;
    std::unique_ptr<SecureTelemetryLogger> telemetry_logger_;
    std::unique_ptr<MLTemplateManager> template_manager_;
    std::unique_ptr<SupernovaAPIConnector> api_connector_;
    std::unique_ptr<MLPerformanceMonitor> performance_monitor_;
    std::unique_ptr<BankingMLValidator> banking_validator_;

    // Base interface integration
    std::shared_ptr<SupernovaMLInterface> base_interface_;

    // Configuration and state
    AdvancedMLConfig config_;
    mutable std::mutex config_mutex_;
    std::atomic<bool> initialized_{false};
    std::atomic<bool> banking_compliance_enabled_{false};

    // Asynchronous processing
    std::mutex async_mutex_;
    std::condition_variable async_cv_;
    std::queue<std::function<void()>> async_queue_;
    std::vector<std::thread> worker_threads_;
    std::atomic<bool> shutdown_requested_{false};

    // Performance monitoring
    mutable std::mutex stats_mutex_;
    std::atomic<size_t> total_ml_requests_{0};
    std::atomic<size_t> successful_ml_requests_{0};
    std::atomic<size_t> failed_ml_requests_{0};
    std::atomic<double> total_ml_processing_time_{0.0};
    std::atomic<float> cumulative_ml_accuracy_{0.0f};

    // Error handling
    mutable std::mutex error_mutex_;
    std::string last_ml_error_;
    std::vector<std::string> error_history_;

    /**
     * @brief Initialize all ML components
     * @return FaceResultCode indicating component initialization result
     */
    FaceResultCode initializeMLComponents();

    /**
     * @brief Process single ML request with full pipeline
     * @param request ML processing request
     * @param result Output ML result
     * @return FaceResultCode indicating processing result
     */
    FaceResultCode processMLRequestInternal(const MLProcessingRequest& request,
                                          AdvancedMLResult& result);

    /**
     * @brief Enhance template features using ML
     * @param template_data Input template
     * @param enhanced_features Output enhanced features
     * @return FaceResultCode indicating enhancement result
     */
    FaceResultCode enhanceTemplateFeatures(const BiometricTemplate& template_data,
                                          std::vector<float>& enhanced_features);

    /**
     * @brief Validate ML processing quality
     * @param request Processing request
     * @param result Processing result
     * @return True if quality meets requirements
     */
    bool validateMLProcessingQuality(const MLProcessingRequest& request,
                                   const AdvancedMLResult& result);

    /**
     * @brief Update ML performance metrics
     * @param processing_time Processing time in milliseconds
     * @param success Whether operation was successful
     * @param accuracy Accuracy score if available
     */
    void updateMLMetrics(double processing_time, bool success, float accuracy = -1.0f);

    /**
     * @brief Log ML processing event for audit trail
     * @param event_type Type of ML event
     * @param request_data Request data summary
     * @param result_data Result data summary
     * @param user_id Associated user ID
     */
    void logMLProcessingEvent(const std::string& event_type,
                             const std::map<std::string, std::string>& request_data,
                             const std::map<std::string, std::string>& result_data,
                             const std::string& user_id = "");

    /**
     * @brief Worker thread function for asynchronous processing
     */
    void asyncWorkerThread();

    /**
     * @brief Set last ML error message
     * @param error_message Error message to set
     */
    void setLastMLError(const std::string& error_message);

    // Disable copy constructor and assignment
    SupernovaMLClient(const SupernovaMLClient&) = delete;
    SupernovaMLClient& operator=(const SupernovaMLClient&) = delete;

    // Enable move semantics
    SupernovaMLClient(SupernovaMLClient&&) noexcept;
    SupernovaMLClient& operator=(SupernovaMLClient&&) noexcept;
};

/**
 * @brief SupernovaMLClient utility functions
 */
class SupernovaMLClientUtils {
public:
    /**
     * @brief Validate advanced ML configuration
     * @param config Configuration to validate
     * @param validation_errors Output validation errors
     * @return True if configuration is valid
     */
    static bool validateAdvancedMLConfig(const AdvancedMLConfig& config,
                                        std::vector<std::string>& validation_errors);

    /**
     * @brief Generate ML processing performance report
     * @param results Vector of ML processing results
     * @param include_recommendations Include performance recommendations
     * @return Formatted performance report
     */
    static std::string generateMLPerformanceReport(
        const std::vector<AdvancedMLResult>& results,
        bool include_recommendations = true);

    /**
     * @brief Estimate ML processing resource requirements
     * @param config ML configuration
     * @param expected_load Expected processing load
     * @return Resource requirement estimates
     */
    static std::map<std::string, std::string> estimateMLResourceRequirements(
        const AdvancedMLConfig& config,
        const std::map<std::string, float>& expected_load);

    /**
     * @brief Generate banking compliance summary
     * @param ml_results Vector of ML results
     * @param compliance_requirements Compliance requirements
     * @return Banking compliance summary report
     */
    static std::string generateBankingComplianceSummary(
        const std::vector<AdvancedMLResult>& ml_results,
        const std::map<std::string, std::string>& compliance_requirements);
};

} // namespace face
} // namespace unlook