#pragma once

/**
 * @file MLTemplateManager.hpp
 * @brief ML Template Manager for Banking-Grade Template Lifecycle Management
 *
 * Professional template management system providing comprehensive lifecycle
 * management for banking-grade biometric templates with optimization,
 * caching, quality assessment, and secure storage capabilities.
 *
 * @version 2.0.0-banking-templates
 * @author Unlook Team - API Architecture Agent
 * @copyright 2025 Unlook. All rights reserved.
 */

#include "unlook/face/FaceTypes.hpp"
#include "unlook/face/SupernovaMLClient.hpp"
#include "unlook/core/types.hpp"
#include "unlook/core/Logger.hpp"

#include <memory>
#include <string>
#include <vector>
#include <map>
#include <mutex>
#include <atomic>
#include <chrono>
#include <unordered_map>
#include <queue>

namespace unlook {
namespace face {

/**
 * @brief ML template management configuration
 */
struct MLTemplateManagerConfig {
    // Core template management settings
    bool enable_template_optimization = true;      ///< Enable template optimization
    bool enable_template_caching = true;           ///< Enable template caching
    bool enable_quality_assessment = true;         ///< Enable quality assessment
    bool enable_version_control = true;            ///< Enable template versioning

    // Optimization settings
    std::string optimization_algorithm = "ml_enhanced"; ///< Optimization algorithm
    bool enable_multi_template_fusion = true;      ///< Enable multi-template fusion
    bool enable_quality_filtering = true;          ///< Enable quality-based filtering
    float quality_threshold = 0.8f;                ///< Minimum quality threshold
    int max_templates_per_user = 5;               ///< Maximum templates per user

    // Caching settings
    size_t max_cache_size = 10000;                 ///< Maximum cache size
    int cache_ttl_hours = 24;                     ///< Cache time-to-live (hours)
    bool enable_lru_eviction = true;              ///< Enable LRU cache eviction
    bool enable_cache_compression = true;          ///< Enable cache compression

    // Storage settings
    std::string storage_backend = "filesystem";    ///< Storage backend type
    std::string storage_path = "./templates";      ///< Template storage path
    bool enable_encrypted_storage = true;          ///< Enable encrypted storage
    bool enable_backup_storage = true;             ///< Enable backup storage
    int backup_retention_days = 30;               ///< Backup retention period

    // Banking compliance settings
    bool banking_grade_templates = false;          ///< Enable banking-grade templates
    bool require_template_signing = true;          ///< Require digital signatures
    bool enable_audit_trail = true;               ///< Enable audit trail
    std::string compliance_level = "PCI_DSS_L1";  ///< Compliance level

    // Performance settings
    bool enable_async_operations = true;           ///< Enable async operations
    int max_worker_threads = 4;                   ///< Maximum worker threads
    bool enable_batch_processing = true;           ///< Enable batch processing
    int batch_size = 100;                         ///< Batch processing size

    bool validate() const;
    std::string toString() const;
};

/**
 * @brief Template lifecycle stages
 */
enum class TemplateLifecycleStage {
    CREATED,           ///< Template created
    OPTIMIZED,         ///< Template optimized
    VALIDATED,         ///< Template validated
    ENROLLED,          ///< Template enrolled
    ACTIVE,            ///< Template active
    DEPRECATED,        ///< Template deprecated
    ARCHIVED,          ///< Template archived
    DELETED            ///< Template deleted
};

/**
 * @brief Template metadata for lifecycle management
 */
struct TemplateMetadata {
    // Basic information
    std::string template_id;                       ///< Unique template identifier
    std::string user_id;                           ///< Associated user identifier
    std::string version;                           ///< Template version
    std::chrono::system_clock::time_point created_at; ///< Creation timestamp
    std::chrono::system_clock::time_point updated_at; ///< Last update timestamp

    // Lifecycle information
    TemplateLifecycleStage lifecycle_stage;        ///< Current lifecycle stage
    std::string lifecycle_history;                 ///< Lifecycle history
    int usage_count = 0;                          ///< Number of times used
    std::chrono::system_clock::time_point last_used; ///< Last usage timestamp

    // Quality information
    float quality_score = 0.0f;                   ///< Overall quality score
    float enrollment_quality = 0.0f;              ///< Enrollment quality
    float verification_accuracy = 0.0f;           ///< Historical verification accuracy
    std::map<std::string, float> quality_metrics; ///< Detailed quality metrics

    // Technical information
    size_t template_size_bytes = 0;               ///< Template size in bytes
    std::string feature_version;                   ///< Feature extraction version
    std::string optimization_method;               ///< Optimization method used
    bool is_optimized = false;                    ///< Optimization status

    // Security information
    bool is_encrypted = false;                    ///< Encryption status
    std::string encryption_key_id;                ///< Encryption key identifier
    std::string digital_signature;                ///< Digital signature
    bool integrity_verified = false;              ///< Integrity verification status

    // Compliance information
    std::string compliance_level;                  ///< Compliance level achieved
    bool gdpr_compliant = false;                  ///< GDPR compliance status
    bool banking_compliant = false;               ///< Banking compliance status
    std::map<std::string, std::string> compliance_metadata; ///< Compliance metadata

    TemplateMetadata() {
        auto now = std::chrono::system_clock::now();
        created_at = now;
        updated_at = now;
        last_used = now;
        lifecycle_stage = TemplateLifecycleStage::CREATED;
    }
};

/**
 * @brief Template optimization result
 */
struct TemplateOptimizationResult {
    bool success = false;                          ///< Optimization success flag
    BiometricTemplate optimized_template;          ///< Optimized template
    TemplateMetadata optimized_metadata;           ///< Optimized template metadata

    // Optimization metrics
    float quality_improvement = 0.0f;             ///< Quality improvement achieved
    float size_reduction = 0.0f;                  ///< Size reduction achieved
    double optimization_time_ms = 0.0;            ///< Optimization processing time

    // Optimization details
    std::string optimization_method;               ///< Method used for optimization
    std::vector<std::string> optimizations_applied; ///< List of optimizations applied
    std::map<std::string, float> before_metrics;  ///< Metrics before optimization
    std::map<std::string, float> after_metrics;   ///< Metrics after optimization

    TemplateOptimizationResult() = default;
};

/**
 * @brief ML Template Manager for Banking-Grade Template Lifecycle Management
 *
 * Professional template management system providing comprehensive lifecycle
 * management for banking-grade biometric templates.
 *
 * Key Features:
 * - Complete template lifecycle management (create, optimize, validate, archive)
 * - ML-based template optimization and quality enhancement
 * - Secure encrypted storage with digital signatures
 * - Performance-optimized caching with LRU eviction
 * - Multi-template fusion for improved accuracy
 * - Banking compliance with comprehensive audit trail
 * - Version control and rollback capabilities
 * - Batch processing for high-throughput scenarios
 * - Quality-based template filtering and selection
 * - Integration with external storage backends
 *
 * Banking Compliance:
 * - PCI DSS Level 1 compliance for payment applications
 * - GDPR compliance for personal data protection
 * - SOX compliance for financial reporting
 * - Digital signatures for template integrity
 * - Comprehensive audit trail for all operations
 * - Secure key management for encryption
 */
class MLTemplateManager {
public:
    /**
     * @brief Constructor with default configuration
     */
    MLTemplateManager();

    /**
     * @brief Constructor with custom configuration
     * @param config ML template manager configuration
     */
    explicit MLTemplateManager(const MLTemplateManagerConfig& config);

    /**
     * @brief Destructor
     */
    ~MLTemplateManager();

    /**
     * @brief Initialize ML template manager
     * @param ml_config ML configuration for compliance settings
     * @return True if initialization successful
     */
    bool initialize(const AdvancedMLConfig& ml_config);

    /**
     * @brief Check if template manager is initialized and ready
     * @return True if ready for template operations
     */
    bool isInitialized() const;

    /**
     * @brief Optimize enrollment template using ML algorithms
     * @param enrollment_data Vector of enrollment templates
     * @param user_context User enrollment context
     * @param optimized_template Output optimized template
     * @return FaceResultCode indicating optimization result
     */
    FaceResultCode optimizeEnrollmentTemplate(
        const std::vector<BiometricTemplate>& enrollment_data,
        const std::map<std::string, std::string>& user_context,
        BiometricTemplate& optimized_template);

    /**
     * @brief Optimize enrollment template with detailed results
     * @param enrollment_data Vector of enrollment templates
     * @param user_context User enrollment context
     * @param optimization_result Output detailed optimization result
     * @return FaceResultCode indicating optimization result
     */
    FaceResultCode optimizeEnrollmentTemplateDetailed(
        const std::vector<BiometricTemplate>& enrollment_data,
        const std::map<std::string, std::string>& user_context,
        TemplateOptimizationResult& optimization_result);

    /**
     * @brief Store template with metadata and lifecycle management
     * @param template_data Template to store
     * @param metadata Template metadata
     * @param template_id Output assigned template identifier
     * @return FaceResultCode indicating storage result
     */
    FaceResultCode storeTemplate(const BiometricTemplate& template_data,
                                const TemplateMetadata& metadata,
                                std::string& template_id);

    /**
     * @brief Retrieve template by identifier
     * @param template_id Template identifier
     * @param template_data Output template data
     * @param metadata Output template metadata
     * @return FaceResultCode indicating retrieval result
     */
    FaceResultCode retrieveTemplate(const std::string& template_id,
                                   BiometricTemplate& template_data,
                                   TemplateMetadata& metadata);

    /**
     * @brief Update existing template
     * @param template_id Template identifier
     * @param template_data Updated template data
     * @param metadata Updated template metadata
     * @return FaceResultCode indicating update result
     */
    FaceResultCode updateTemplate(const std::string& template_id,
                                 const BiometricTemplate& template_data,
                                 const TemplateMetadata& metadata);

    /**
     * @brief Delete template (with compliance considerations)
     * @param template_id Template identifier
     * @param deletion_reason Reason for deletion
     * @param force_delete Force deletion without retention
     * @return FaceResultCode indicating deletion result
     */
    FaceResultCode deleteTemplate(const std::string& template_id,
                                 const std::string& deletion_reason = "",
                                 bool force_delete = false);

    /**
     * @brief List templates for user with filtering options
     * @param user_id User identifier
     * @param template_list Output list of template metadata
     * @param filter_criteria Optional filtering criteria
     * @return Number of templates found
     */
    size_t listUserTemplates(const std::string& user_id,
                            std::vector<TemplateMetadata>& template_list,
                            const std::map<std::string, std::string>& filter_criteria = {});

    /**
     * @brief Assess template quality using ML algorithms
     * @param template_data Template to assess
     * @param quality_metrics Output detailed quality metrics
     * @return Quality score (0-1)
     */
    float assessTemplateQuality(const BiometricTemplate& template_data,
                               std::map<std::string, float>& quality_metrics);

    /**
     * @brief Fuse multiple templates for improved accuracy
     * @param templates Vector of templates to fuse
     * @param fusion_method Fusion method ("average", "weighted", "ml_enhanced")
     * @param fused_template Output fused template
     * @return FaceResultCode indicating fusion result
     */
    FaceResultCode fuseTemplates(const std::vector<BiometricTemplate>& templates,
                                const std::string& fusion_method,
                                BiometricTemplate& fused_template);

    /**
     * @brief Banking-grade template validation
     * @param template_data Template to validate
     * @param validation_context Banking validation context
     * @param validation_result Output validation result with compliance status
     * @return True if template meets banking requirements
     */
    bool validateBankingGradeTemplate(const BiometricTemplate& template_data,
                                     const std::map<std::string, std::string>& validation_context,
                                     std::map<std::string, std::string>& validation_result);

    /**
     * @brief Update template lifecycle stage
     * @param template_id Template identifier
     * @param new_stage New lifecycle stage
     * @param stage_context Context information for stage change
     * @return FaceResultCode indicating update result
     */
    FaceResultCode updateTemplateLifecycleStage(const std::string& template_id,
                                               TemplateLifecycleStage new_stage,
                                               const std::map<std::string, std::string>& stage_context = {});

    /**
     * @brief Get template usage statistics
     * @param template_id Template identifier
     * @param usage_stats Output usage statistics
     * @return FaceResultCode indicating retrieval result
     */
    FaceResultCode getTemplateUsageStatistics(const std::string& template_id,
                                             std::map<std::string, int>& usage_stats);

    /**
     * @brief Batch template processing
     * @param operation Operation to perform ("optimize", "validate", "assess_quality")
     * @param templates Vector of templates to process
     * @param results Output vector of processing results
     * @param progress_callback Optional progress callback (0-100%)
     * @return Number of successfully processed templates
     */
    size_t batchProcessTemplates(const std::string& operation,
                                const std::vector<BiometricTemplate>& templates,
                                std::vector<std::map<std::string, std::string>>& results,
                                std::function<void(float, size_t, size_t)> progress_callback = nullptr);

    /**
     * @brief Export template data for backup or migration
     * @param template_ids Vector of template IDs to export
     * @param output_path Export output path
     * @param export_format Export format ("json", "binary", "encrypted")
     * @param include_metadata Include metadata in export
     * @return FaceResultCode indicating export result
     */
    FaceResultCode exportTemplates(const std::vector<std::string>& template_ids,
                                  const std::string& output_path,
                                  const std::string& export_format = "encrypted",
                                  bool include_metadata = true);

    /**
     * @brief Import template data from backup or migration
     * @param input_path Import input path
     * @param import_format Import format ("json", "binary", "encrypted")
     * @param imported_template_ids Output vector of imported template IDs
     * @return FaceResultCode indicating import result
     */
    FaceResultCode importTemplates(const std::string& input_path,
                                  const std::string& import_format = "encrypted",
                                  std::vector<std::string>& imported_template_ids);

    /**
     * @brief Set template manager configuration
     * @param config New configuration
     * @return True if configuration applied successfully
     */
    bool setConfiguration(const MLTemplateManagerConfig& config);

    /**
     * @brief Get current template manager configuration
     * @return Current configuration
     */
    MLTemplateManagerConfig getConfiguration() const;

    /**
     * @brief Enable/disable specific template management features
     * @param enable_optimization Enable template optimization
     * @param enable_caching Enable template caching
     * @param enable_quality_assessment Enable quality assessment
     * @param enable_versioning Enable version control
     */
    void setTemplateFeatures(bool enable_optimization,
                            bool enable_caching,
                            bool enable_quality_assessment,
                            bool enable_versioning);

    /**
     * @brief Set banking compliance mode
     * @param enable Enable banking compliance
     * @param compliance_level Compliance level
     * @param require_signing Require digital signatures
     */
    void setBankingComplianceMode(bool enable,
                                 const std::string& compliance_level = "PCI_DSS_L1",
                                 bool require_signing = true);

    /**
     * @brief Get template management statistics
     * @param total_templates Total templates managed
     * @param cached_templates Cached templates count
     * @param optimized_templates Optimized templates count
     * @param avg_quality_score Average quality score
     * @param storage_usage_mb Storage usage (MB)
     */
    void getManagementStatistics(size_t& total_templates,
                                size_t& cached_templates,
                                size_t& optimized_templates,
                                float& avg_quality_score,
                                float& storage_usage_mb) const;

    /**
     * @brief Perform template maintenance (cleanup, optimization, archival)
     * @param maintenance_type Type of maintenance ("cleanup", "optimize", "archive")
     * @param maintenance_options Maintenance options
     * @return FaceResultCode indicating maintenance result
     */
    FaceResultCode performTemplateMaintenance(const std::string& maintenance_type,
                                             const std::map<std::string, std::string>& maintenance_options = {});

    /**
     * @brief Get last template management error
     * @return Human-readable error message
     */
    std::string getLastError() const;

    /**
     * @brief Validate template management system integrity
     * @param integrity_report Output integrity validation report
     * @return True if template system is functioning correctly
     */
    bool validateSystemIntegrity(std::map<std::string, std::string>& integrity_report);

private:
    // Configuration and state
    MLTemplateManagerConfig config_;
    mutable std::mutex config_mutex_;
    std::atomic<bool> initialized_{false};
    std::atomic<bool> banking_compliance_enabled_{false};

    // Template storage
    std::mutex storage_mutex_;
    std::unordered_map<std::string, BiometricTemplate> template_storage_;
    std::unordered_map<std::string, TemplateMetadata> metadata_storage_;

    // Template caching
    struct CacheEntry {
        BiometricTemplate template_data;
        TemplateMetadata metadata;
        std::chrono::system_clock::time_point last_accessed;
        int access_count = 0;
    };

    std::mutex cache_mutex_;
    std::unordered_map<std::string, CacheEntry> template_cache_;
    std::queue<std::string> lru_queue_;
    size_t current_cache_size_ = 0;

    // Performance monitoring
    mutable std::mutex stats_mutex_;
    std::atomic<size_t> total_operations_{0};
    std::atomic<size_t> successful_operations_{0};
    std::atomic<size_t> failed_operations_{0};
    std::atomic<double> total_processing_time_{0.0};
    std::atomic<float> cumulative_quality_score_{0.0f};

    // Error handling
    mutable std::mutex error_mutex_;
    std::string last_error_;
    std::vector<std::string> error_history_;

    /**
     * @brief Initialize template management components
     * @return True if initialization successful
     */
    bool initializeComponents();

    /**
     * @brief Initialize secure storage backend
     * @return True if storage initialization successful
     */
    bool initializeStorage();

    /**
     * @brief Optimize single template using ML algorithms
     * @param template_data Input template
     * @param optimization_params Optimization parameters
     * @param optimized_template Output optimized template
     * @return FaceResultCode indicating optimization result
     */
    FaceResultCode optimizeSingleTemplate(const BiometricTemplate& template_data,
                                         const std::map<std::string, float>& optimization_params,
                                         BiometricTemplate& optimized_template);

    /**
     * @brief Apply template quality filtering
     * @param templates Input templates
     * @param quality_threshold Minimum quality threshold
     * @param filtered_templates Output filtered templates
     * @return Number of templates that passed filtering
     */
    size_t applyQualityFiltering(const std::vector<BiometricTemplate>& templates,
                                float quality_threshold,
                                std::vector<BiometricTemplate>& filtered_templates);

    /**
     * @brief Generate unique template identifier
     * @param user_id User identifier
     * @return Unique template identifier
     */
    std::string generateTemplateId(const std::string& user_id);

    /**
     * @brief Update template cache with LRU eviction
     * @param template_id Template identifier
     * @param template_data Template data
     * @param metadata Template metadata
     */
    void updateCache(const std::string& template_id,
                    const BiometricTemplate& template_data,
                    const TemplateMetadata& metadata);

    /**
     * @brief Evict least recently used templates from cache
     */
    void evictLRUFromCache();

    /**
     * @brief Encrypt template data for secure storage
     * @param template_data Plain template data
     * @param encrypted_data Output encrypted template data
     * @return True if encryption successful
     */
    bool encryptTemplateData(const BiometricTemplate& template_data,
                            std::vector<uint8_t>& encrypted_data);

    /**
     * @brief Decrypt template data from secure storage
     * @param encrypted_data Encrypted template data
     * @param template_data Output plain template data
     * @return True if decryption successful
     */
    bool decryptTemplateData(const std::vector<uint8_t>& encrypted_data,
                            BiometricTemplate& template_data);

    /**
     * @brief Generate digital signature for template
     * @param template_data Template data
     * @param signature Output digital signature
     * @return True if signature generation successful
     */
    bool generateTemplateSignature(const BiometricTemplate& template_data,
                                  std::string& signature);

    /**
     * @brief Verify digital signature of template
     * @param template_data Template data
     * @param signature Digital signature to verify
     * @return True if signature is valid
     */
    bool verifyTemplateSignature(const BiometricTemplate& template_data,
                                const std::string& signature);

    /**
     * @brief Update template management statistics
     * @param processing_time Processing time
     * @param success Whether operation was successful
     * @param quality_score Quality score if available
     */
    void updateStatistics(double processing_time, bool success, float quality_score = -1.0f);

    /**
     * @brief Set last error message
     * @param error_message Error message to set
     */
    void setLastError(const std::string& error_message);

    // Disable copy constructor and assignment
    MLTemplateManager(const MLTemplateManager&) = delete;
    MLTemplateManager& operator=(const MLTemplateManager&) = delete;

    // Enable move semantics
    MLTemplateManager(MLTemplateManager&&) noexcept;
    MLTemplateManager& operator=(MLTemplateManager&&) noexcept;
};

/**
 * @brief ML Template Manager utility functions
 */
class MLTemplateManagerUtils {
public:
    /**
     * @brief Validate ML template manager configuration
     * @param config Configuration to validate
     * @param validation_errors Output validation errors
     * @return True if configuration is valid
     */
    static bool validateConfiguration(const MLTemplateManagerConfig& config,
                                     std::vector<std::string>& validation_errors);

    /**
     * @brief Generate template management performance report
     * @param optimization_results Vector of optimization results
     * @param include_recommendations Include performance recommendations
     * @return Formatted performance report
     */
    static std::string generatePerformanceReport(
        const std::vector<TemplateOptimizationResult>& optimization_results,
        bool include_recommendations = true);

    /**
     * @brief Estimate storage requirements for template management
     * @param config Template manager configuration
     * @param expected_templates Expected number of templates
     * @return Storage requirement estimates
     */
    static std::map<std::string, std::string> estimateStorageRequirements(
        const MLTemplateManagerConfig& config,
        size_t expected_templates);

    /**
     * @brief Convert template metadata to JSON format
     * @param metadata Template metadata
     * @param include_sensitive Include sensitive information
     * @return JSON representation of metadata
     */
    static std::string templateMetadataToJson(const TemplateMetadata& metadata,
                                             bool include_sensitive = false);

    /**
     * @brief Parse template metadata from JSON format
     * @param json_data JSON data to parse
     * @param metadata Output template metadata
     * @return True if parsing successful
     */
    static bool templateMetadataFromJson(const std::string& json_data,
                                        TemplateMetadata& metadata);
};

} // namespace face
} // namespace unlook