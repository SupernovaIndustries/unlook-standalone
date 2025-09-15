#pragma once

#include "unlook/face/FaceTypes.hpp"
#include <opencv2/opencv.hpp>
#include <memory>
#include <vector>
#include <string>
#include <map>
#include <functional>
#include <mutex>
#include <atomic>
#include <future>

namespace unlook {
namespace face {

/**
 * @brief Supernova ML integration configuration
 */
struct SupernovaMLConfig {
    // Connection settings
    std::string server_endpoint = "https://api.supernova-ml.com";
    std::string api_version = "v2.0";
    std::string client_id;                    ///< Client identifier
    std::string client_secret;               ///< Client secret for authentication
    std::string device_id;                   ///< Unique device identifier

    // API settings
    int timeout_seconds = 30;                ///< Request timeout
    int max_retries = 3;                     ///< Maximum retry attempts
    bool enable_ssl_verification = true;     ///< Enable SSL certificate verification
    std::string ssl_cert_path;               ///< Custom SSL certificate path

    // Template settings
    std::string template_format = "unlook_v1"; ///< Template format identifier
    bool enable_template_compression = true;  ///< Enable template compression
    bool enable_encryption = true;           ///< Enable end-to-end encryption
    std::string encryption_key_id;           ///< Encryption key identifier

    // Performance settings
    bool enable_caching = true;              ///< Enable response caching
    int cache_ttl_seconds = 3600;           ///< Cache time-to-live
    bool enable_batch_operations = true;    ///< Enable batch processing
    int max_batch_size = 100;               ///< Maximum batch size

    // Banking compliance
    bool banking_mode = false;               ///< Enable banking compliance mode
    bool enable_audit_logging = true;       ///< Enable comprehensive audit logs
    bool require_transaction_ids = true;    ///< Require transaction IDs
    std::string compliance_level = "PCI_DSS"; ///< Compliance level

    bool validate() const;
    std::string toString() const;
};

/**
 * @brief Supernova ML service response
 */
struct SupernovaMLResponse {
    // Response status
    bool success = false;                    ///< Operation success flag
    int status_code = 0;                     ///< HTTP status code
    std::string status_message;              ///< Status message
    std::string transaction_id;              ///< Unique transaction ID

    // Response data
    std::map<std::string, std::string> metadata; ///< Response metadata
    std::vector<uint8_t> data;               ///< Response data payload

    // Timing information
    double request_time_ms = 0.0;            ///< Request processing time
    std::chrono::system_clock::time_point timestamp; ///< Response timestamp

    // Error information
    std::string error_code;                  ///< Error code if applicable
    std::string error_message;               ///< Detailed error message
    std::vector<std::string> validation_errors; ///< Validation error details

    SupernovaMLResponse() : timestamp(std::chrono::system_clock::now()) {}

    bool isSuccessful() const { return success && status_code >= 200 && status_code < 300; }
    bool hasError() const { return !success || status_code >= 400; }
};

/**
 * @brief Template verification result from Supernova ML
 */
struct SupernovaVerificationResult {
    bool is_match = false;                   ///< Verification result
    float similarity_score = 0.0f;          ///< Similarity score (0-1)
    float confidence_level = 0.0f;          ///< Confidence in result (0-1)

    // Banking-grade metrics
    float false_accept_rate = 1.0f;          ///< Estimated FAR for this match
    float false_reject_rate = 1.0f;          ///< Estimated FRR for this match
    bool banking_grade_match = false;        ///< Banking grade match flag

    // Quality assessment
    float template_quality_score = 0.0f;     ///< Template quality assessment
    float probe_quality_score = 0.0f;        ///< Probe quality assessment

    // Compliance information
    std::string compliance_level;            ///< Achieved compliance level
    bool audit_trail_complete = false;       ///< Audit trail completeness
    std::string verification_method;         ///< Method used for verification

    // Performance metrics
    double processing_time_ms = 0.0;         ///< ML processing time
    std::string model_version;               ///< ML model version used

    SupernovaVerificationResult() = default;
};

/**
 * @brief Supernova ML integration interface for banking-grade facial recognition
 *
 * Professional interface to Supernova ML cloud services for enhanced
 * facial recognition accuracy and banking compliance. Provides secure
 * template exchange, cloud-based matching, and compliance reporting.
 *
 * Key features:
 * - Secure template upload and storage in Supernova cloud
 * - Enhanced matching accuracy through cloud ML models
 * - Banking-grade compliance and audit trail
 * - Real-time and batch processing support
 * - End-to-end encryption and secure communication
 * - Performance monitoring and analytics
 * - Fallback to local processing if cloud unavailable
 * - ARM64/CM4 optimized networking and processing
 *
 * Integrates seamlessly with Unlook face recognition pipeline.
 */
class SupernovaMLInterface {
public:
    /**
     * @brief Constructor with default configuration
     */
    SupernovaMLInterface();

    /**
     * @brief Constructor with custom configuration
     * @param config Supernova ML configuration
     */
    explicit SupernovaMLInterface(const SupernovaMLConfig& config);

    /**
     * @brief Destructor
     */
    ~SupernovaMLInterface();

    /**
     * @brief Initialize interface with authentication
     * @param config Configuration parameters
     * @return FaceResultCode indicating success or failure
     */
    FaceResultCode initialize(const SupernovaMLConfig& config);

    /**
     * @brief Check if interface is initialized and connected
     * @return True if ready for operations
     */
    bool isInitialized() const;

    /**
     * @brief Test connection to Supernova ML services
     * @param response Output response with connection details
     * @return FaceResultCode indicating connection status
     */
    FaceResultCode testConnection(SupernovaMLResponse& response);

    /**
     * @brief Upload biometric template to Supernova cloud
     * @param template_data Unlook biometric template
     * @param user_id User identifier
     * @param template_id Unique template identifier
     * @param response Output upload response
     * @return FaceResultCode indicating success or failure
     */
    FaceResultCode uploadTemplate(const BiometricTemplate& template_data,
                                 const std::string& user_id,
                                 const std::string& template_id,
                                 SupernovaMLResponse& response);

    /**
     * @brief Perform 1:1 verification using Supernova ML
     * @param probe_template Probe template to verify
     * @param reference_template_id Reference template ID in cloud
     * @param verification_result Output verification result
     * @return FaceResultCode indicating success or failure
     */
    FaceResultCode verifyTemplate(const BiometricTemplate& probe_template,
                                 const std::string& reference_template_id,
                                 SupernovaVerificationResult& verification_result);

    /**
     * @brief Perform 1:N identification using Supernova ML
     * @param probe_template Probe template to identify
     * @param database_id Database identifier for search
     * @param max_candidates Maximum candidates to return
     * @param identification_results Output identification results
     * @return FaceResultCode indicating success or failure
     */
    FaceResultCode identifyTemplate(const BiometricTemplate& probe_template,
                                   const std::string& database_id,
                                   int max_candidates,
                                   std::vector<SupernovaVerificationResult>& identification_results);

    /**
     * @brief Banking-grade verification with enhanced security
     * @param probe_template Probe template
     * @param reference_template_id Reference template ID
     * @param transaction_context Banking transaction context
     * @param banking_result Output banking-grade result
     * @return FaceResultCode indicating success or failure
     */
    FaceResultCode verifyBankingGrade(const BiometricTemplate& probe_template,
                                     const std::string& reference_template_id,
                                     const std::map<std::string, std::string>& transaction_context,
                                     SupernovaVerificationResult& banking_result);

    /**
     * @brief Asynchronous template verification
     * @param probe_template Probe template
     * @param reference_template_id Reference template ID
     * @param callback Callback function for result
     * @return Future for async operation
     */
    std::future<SupernovaVerificationResult> verifyTemplateAsync(
        const BiometricTemplate& probe_template,
        const std::string& reference_template_id);

    /**
     * @brief Batch template upload
     * @param templates Vector of templates to upload
     * @param user_ids Corresponding user identifiers
     * @param template_ids Corresponding template identifiers
     * @param progress_callback Optional progress callback (0-100)
     * @return Number of successfully uploaded templates
     */
    size_t uploadTemplatesBatch(const std::vector<BiometricTemplate>& templates,
                               const std::vector<std::string>& user_ids,
                               const std::vector<std::string>& template_ids,
                               std::function<void(float)> progress_callback = nullptr);

    /**
     * @brief Download template from Supernova cloud
     * @param template_id Template identifier to download
     * @param template_data Output template data
     * @param response Output download response
     * @return FaceResultCode indicating success or failure
     */
    FaceResultCode downloadTemplate(const std::string& template_id,
                                   BiometricTemplate& template_data,
                                   SupernovaMLResponse& response);

    /**
     * @brief Delete template from Supernova cloud
     * @param template_id Template identifier to delete
     * @param response Output deletion response
     * @return FaceResultCode indicating success or failure
     */
    FaceResultCode deleteTemplate(const std::string& template_id,
                                 SupernovaMLResponse& response);

    /**
     * @brief Update existing template in cloud
     * @param template_id Template identifier to update
     * @param new_template_data New template data
     * @param response Output update response
     * @return FaceResultCode indicating success or failure
     */
    FaceResultCode updateTemplate(const std::string& template_id,
                                 const BiometricTemplate& new_template_data,
                                 SupernovaMLResponse& response);

    /**
     * @brief Get template metadata from cloud
     * @param template_id Template identifier
     * @param metadata Output template metadata
     * @param response Output response
     * @return FaceResultCode indicating success or failure
     */
    FaceResultCode getTemplateMetadata(const std::string& template_id,
                                      std::map<std::string, std::string>& metadata,
                                      SupernovaMLResponse& response);

    /**
     * @brief List templates for user in cloud
     * @param user_id User identifier
     * @param template_list Output list of template IDs
     * @param response Output response
     * @return FaceResultCode indicating success or failure
     */
    FaceResultCode listUserTemplates(const std::string& user_id,
                                    std::vector<std::string>& template_list,
                                    SupernovaMLResponse& response);

    /**
     * @brief Get Supernova ML service status and health
     * @param service_status Output service status information
     * @param response Output response
     * @return FaceResultCode indicating success or failure
     */
    FaceResultCode getServiceStatus(std::map<std::string, std::string>& service_status,
                                   SupernovaMLResponse& response);

    /**
     * @brief Set configuration parameters
     * @param config New configuration
     * @return FaceResultCode indicating success or failure
     */
    FaceResultCode setConfiguration(const SupernovaMLConfig& config);

    /**
     * @brief Get current configuration
     * @return Current configuration
     */
    SupernovaMLConfig getConfiguration() const;

    /**
     * @brief Enable/disable specific features
     * @param enable_caching Enable response caching
     * @param enable_encryption Enable end-to-end encryption
     * @param enable_compression Enable template compression
     * @param enable_batch Enable batch operations
     */
    void setFeatures(bool enable_caching,
                    bool enable_encryption,
                    bool enable_compression,
                    bool enable_batch);

    /**
     * @brief Get API usage statistics
     * @param total_requests Total API requests made
     * @param successful_requests Successful requests
     * @param failed_requests Failed requests
     * @param average_response_time Average response time (ms)
     * @param cache_hit_rate Cache hit rate (0-1)
     */
    void getStatistics(size_t& total_requests,
                      size_t& successful_requests,
                      size_t& failed_requests,
                      double& average_response_time,
                      float& cache_hit_rate) const;

    /**
     * @brief Reset API usage statistics
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
     * @brief Export audit log for compliance
     * @param output_path Path to export audit log
     * @param format Export format ("json", "csv", "xml")
     * @param date_range Optional date range filter
     * @return FaceResultCode indicating success or failure
     */
    FaceResultCode exportAuditLog(const std::string& output_path,
                                 const std::string& format = "json",
                                 const std::pair<std::string, std::string>& date_range = {});

    /**
     * @brief Validate template compatibility with Supernova ML
     * @param template_data Template to validate
     * @param compatibility_info Output compatibility information
     * @return True if template is compatible
     */
    bool validateTemplateCompatibility(const BiometricTemplate& template_data,
                                      std::string& compatibility_info);

    /**
     * @brief Get supported Supernova ML model versions
     * @param model_versions Output list of supported versions
     * @param response Output response
     * @return FaceResultCode indicating success or failure
     */
    FaceResultCode getSupportedModelVersions(std::vector<std::string>& model_versions,
                                            SupernovaMLResponse& response);

    /**
     * @brief Enable banking-grade mode with strict compliance
     * @param enable Enable/disable banking mode
     * @param compliance_level Compliance level required
     * @param audit_level Audit logging level
     */
    void setBankingMode(bool enable,
                       const std::string& compliance_level = "PCI_DSS",
                       const std::string& audit_level = "FULL");

private:
    class Impl;
    std::unique_ptr<Impl> pImpl;

    // Configuration and state
    SupernovaMLConfig config_;
    mutable std::mutex config_mutex_;
    std::atomic<bool> initialized_{false};

    // Authentication state
    std::mutex auth_mutex_;
    std::string access_token_;
    std::chrono::system_clock::time_point token_expiry_;
    bool has_valid_token_{false};

    // Performance monitoring
    mutable std::mutex stats_mutex_;
    std::atomic<size_t> total_requests_{0};
    std::atomic<size_t> successful_requests_{0};
    std::atomic<size_t> failed_requests_{0};
    std::atomic<double> total_response_time_{0.0};
    std::atomic<size_t> cache_hits_{0};
    std::atomic<size_t> cache_misses_{0};

    // Response cache
    std::mutex cache_mutex_;
    std::map<std::string, std::pair<SupernovaMLResponse, std::chrono::system_clock::time_point>> response_cache_;

    /**
     * @brief Authenticate with Supernova ML services
     * @return FaceResultCode indicating authentication result
     */
    FaceResultCode authenticate();

    /**
     * @brief Refresh access token if needed
     * @return FaceResultCode indicating refresh result
     */
    FaceResultCode refreshTokenIfNeeded();

    /**
     * @brief Convert Unlook template to Supernova format
     * @param unlook_template Input Unlook template
     * @param supernova_data Output Supernova data
     * @param metadata Output metadata
     * @return FaceResultCode indicating conversion result
     */
    FaceResultCode convertToSupernovaFormat(const BiometricTemplate& unlook_template,
                                           std::vector<uint8_t>& supernova_data,
                                           std::map<std::string, std::string>& metadata);

    /**
     * @brief Convert Supernova format to Unlook template
     * @param supernova_data Input Supernova data
     * @param metadata Input metadata
     * @param unlook_template Output Unlook template
     * @return FaceResultCode indicating conversion result
     */
    FaceResultCode convertFromSupernovaFormat(const std::vector<uint8_t>& supernova_data,
                                             const std::map<std::string, std::string>& metadata,
                                             BiometricTemplate& unlook_template);

    /**
     * @brief Make HTTP request to Supernova ML API
     * @param endpoint API endpoint
     * @param method HTTP method ("GET", "POST", "PUT", "DELETE")
     * @param headers Request headers
     * @param data Request data
     * @param response Output response
     * @return FaceResultCode indicating request result
     */
    FaceResultCode makeAPIRequest(const std::string& endpoint,
                                 const std::string& method,
                                 const std::map<std::string, std::string>& headers,
                                 const std::vector<uint8_t>& data,
                                 SupernovaMLResponse& response);

    /**
     * @brief Encrypt data for secure transmission
     * @param plain_data Input plain data
     * @param encrypted_data Output encrypted data
     * @return FaceResultCode indicating encryption result
     */
    FaceResultCode encryptData(const std::vector<uint8_t>& plain_data,
                              std::vector<uint8_t>& encrypted_data);

    /**
     * @brief Decrypt received data
     * @param encrypted_data Input encrypted data
     * @param plain_data Output decrypted data
     * @return FaceResultCode indicating decryption result
     */
    FaceResultCode decryptData(const std::vector<uint8_t>& encrypted_data,
                              std::vector<uint8_t>& plain_data);

    /**
     * @brief Compress template data
     * @param template_data Input template data
     * @param compressed_data Output compressed data
     * @return Compression ratio achieved
     */
    float compressTemplateData(const std::vector<uint8_t>& template_data,
                              std::vector<uint8_t>& compressed_data);

    /**
     * @brief Decompress template data
     * @param compressed_data Input compressed data
     * @param template_data Output decompressed data
     * @return FaceResultCode indicating decompression result
     */
    FaceResultCode decompressTemplateData(const std::vector<uint8_t>& compressed_data,
                                         std::vector<uint8_t>& template_data);

    /**
     * @brief Check response cache for cached result
     * @param cache_key Cache key
     * @param response Output cached response
     * @return True if cache hit
     */
    bool checkCache(const std::string& cache_key, SupernovaMLResponse& response);

    /**
     * @brief Store response in cache
     * @param cache_key Cache key
     * @param response Response to cache
     */
    void storeInCache(const std::string& cache_key, const SupernovaMLResponse& response);

    /**
     * @brief Generate cache key for request
     * @param endpoint API endpoint
     * @param data Request data
     * @return Cache key string
     */
    std::string generateCacheKey(const std::string& endpoint,
                                const std::vector<uint8_t>& data);

    /**
     * @brief Log API request for audit trail
     * @param endpoint API endpoint
     * @param method HTTP method
     * @param data_size Request data size
     * @param response_code Response code
     * @param processing_time Processing time
     */
    void logAPIRequest(const std::string& endpoint,
                      const std::string& method,
                      size_t data_size,
                      int response_code,
                      double processing_time);

    /**
     * @brief Update performance statistics
     * @param request_time Request processing time
     * @param success Whether request was successful
     * @param cache_hit Whether request was served from cache
     */
    void updateStatistics(double request_time, bool success, bool cache_hit);

    /**
     * @brief Clean expired cache entries
     */
    void cleanExpiredCache();

    // Disable copy constructor and assignment
    SupernovaMLInterface(const SupernovaMLInterface&) = delete;
    SupernovaMLInterface& operator=(const SupernovaMLInterface&) = delete;

    // Enable move semantics
    SupernovaMLInterface(SupernovaMLInterface&&) noexcept;
    SupernovaMLInterface& operator=(SupernovaMLInterface&&) noexcept;
};

/**
 * @brief Supernova ML utility functions
 */
class SupernovaMLUtils {
public:
    /**
     * @brief Validate Supernova ML configuration
     * @param config Configuration to validate
     * @param validation_errors Output validation errors
     * @return True if configuration is valid
     */
    static bool validateConfiguration(const SupernovaMLConfig& config,
                                     std::vector<std::string>& validation_errors);

    /**
     * @brief Generate device fingerprint for authentication
     * @param device_info Device information
     * @return Device fingerprint string
     */
    static std::string generateDeviceFingerprint(
        const std::map<std::string, std::string>& device_info);

    /**
     * @brief Parse Supernova ML API response
     * @param raw_response Raw HTTP response
     * @param parsed_response Output parsed response
     * @return True if parsing successful
     */
    static bool parseAPIResponse(const std::string& raw_response,
                                SupernovaMLResponse& parsed_response);

    /**
     * @brief Format template for Supernova ML upload
     * @param template_data Unlook template
     * @param user_metadata User metadata
     * @param formatted_data Output formatted data
     * @return FaceResultCode indicating formatting result
     */
    static FaceResultCode formatTemplateForUpload(
        const BiometricTemplate& template_data,
        const std::map<std::string, std::string>& user_metadata,
        std::vector<uint8_t>& formatted_data);

    /**
     * @brief Generate compliance report
     * @param verification_results Vector of verification results
     * @param compliance_requirements Compliance requirements
     * @return Formatted compliance report
     */
    static std::string generateComplianceReport(
        const std::vector<SupernovaVerificationResult>& verification_results,
        const std::map<std::string, std::string>& compliance_requirements);
};

} // namespace face
} // namespace unlook