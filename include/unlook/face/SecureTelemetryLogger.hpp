#pragma once

/**
 * @file SecureTelemetryLogger.hpp
 * @brief Secure Telemetry Logger for Banking ML Compliance
 *
 * Professional telemetry logging system providing comprehensive audit trail
 * and compliance reporting for banking-grade machine learning operations
 * with encryption, integrity verification, and regulatory compliance.
 *
 * @version 2.0.0-banking-compliance
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
#include <queue>
#include <thread>
#include <condition_variable>
#include <fstream>

namespace unlook {
namespace face {

/**
 * @brief Secure telemetry logging configuration for banking compliance
 */
struct SecureTelemetryConfig {
    // Core logging settings
    bool enable_audit_logging = true;              ///< Enable comprehensive audit logging
    bool enable_real_time_logging = true;          ///< Enable real-time event logging
    bool enable_batch_logging = true;              ///< Enable batch processing
    bool enable_encrypted_storage = true;          ///< Enable encrypted log storage

    // Security settings
    std::string encryption_algorithm = "AES-256-GCM"; ///< Encryption algorithm
    std::string encryption_key_id = "unlook_audit_key"; ///< Encryption key identifier
    bool enable_digital_signatures = true;         ///< Enable digital signatures
    bool enable_integrity_verification = true;     ///< Enable integrity checking
    std::string hash_algorithm = "SHA-256";        ///< Hash algorithm for integrity

    // Compliance settings
    std::string compliance_level = "PCI_DSS_L1";   ///< Compliance level
    bool enable_gdpr_compliance = true;            ///< GDPR compliance mode
    bool enable_hipaa_compliance = false;          ///< HIPAA compliance mode
    bool enable_sox_compliance = false;            ///< SOX compliance mode
    bool anonymize_personal_data = true;           ///< Anonymize personal data

    // Storage settings
    std::string log_directory = "./logs/secure";   ///< Secure log directory
    std::string log_filename_pattern = "audit_%Y%m%d.log"; ///< Log filename pattern
    size_t max_log_file_size_mb = 100;            ///< Maximum log file size (MB)
    int log_retention_days = 2555;                 ///< Log retention period (7 years)
    bool enable_log_rotation = true;               ///< Enable log rotation
    bool enable_log_compression = true;            ///< Enable log compression

    // Performance settings
    size_t log_buffer_size = 10000;               ///< Log buffer size
    int batch_flush_interval_ms = 5000;           ///< Batch flush interval
    int max_worker_threads = 2;                   ///< Maximum worker threads
    bool enable_async_logging = true;             ///< Enable asynchronous logging

    // Monitoring settings
    bool enable_log_monitoring = true;            ///< Enable log monitoring
    bool enable_anomaly_detection = true;         ///< Enable anomaly detection
    bool enable_alert_notifications = true;       ///< Enable alert notifications
    std::string alert_webhook_url;                ///< Alert webhook URL

    bool validate() const;
    std::string toString() const;
};

/**
 * @brief Audit event types for ML operations
 */
enum class AuditEventType {
    ML_VERIFICATION_REQUEST,     ///< ML verification request
    ML_VERIFICATION_RESPONSE,    ///< ML verification response
    TEMPLATE_UPLOAD,             ///< Template upload event
    TEMPLATE_DOWNLOAD,           ///< Template download event
    TEMPLATE_DELETE,             ///< Template deletion event
    AUTHENTICATION_SUCCESS,      ///< Authentication success
    AUTHENTICATION_FAILURE,      ///< Authentication failure
    AUTHORIZATION_GRANT,         ///< Authorization granted
    AUTHORIZATION_DENY,          ///< Authorization denied
    SECURITY_VIOLATION,          ///< Security violation detected
    COMPLIANCE_CHECK,            ///< Compliance check event
    SYSTEM_ERROR,               ///< System error event
    PERFORMANCE_ALERT,          ///< Performance alert
    DATA_EXPORT,                ///< Data export event
    CONFIGURATION_CHANGE        ///< Configuration change
};

/**
 * @brief Comprehensive audit event structure
 */
struct AuditEvent {
    // Event identification
    std::string event_id;                          ///< Unique event identifier
    AuditEventType event_type;                     ///< Type of audit event
    std::chrono::system_clock::time_point timestamp; ///< Event timestamp
    std::string session_id;                        ///< Session identifier
    std::string transaction_id;                    ///< Transaction identifier

    // Event source information
    std::string component_name;                    ///< Component generating event
    std::string function_name;                     ///< Function/method name
    std::string version;                           ///< Component version
    std::string hostname;                          ///< System hostname
    std::string process_id;                        ///< Process identifier

    // User and authentication context
    std::string user_id;                           ///< User identifier (anonymized if needed)
    std::string device_id;                         ///< Device identifier
    std::string client_ip;                         ///< Client IP address (anonymized)
    std::string user_agent;                        ///< User agent string
    std::string authentication_method;             ///< Authentication method used

    // Event data
    std::map<std::string, std::string> event_data; ///< Event-specific data
    std::map<std::string, std::string> metadata;   ///< Additional metadata

    // Security information
    std::string security_level;                    ///< Security level achieved
    bool sensitive_data_present = false;           ///< Sensitive data flag
    std::vector<std::string> security_tags;        ///< Security classification tags

    // Compliance information
    std::string compliance_level;                  ///< Compliance level
    bool gdpr_applicable = false;                  ///< GDPR applicability
    bool hipaa_applicable = false;                 ///< HIPAA applicability
    std::string data_classification;               ///< Data classification level

    // Performance metrics
    double processing_time_ms = 0.0;               ///< Processing time
    size_t data_size_bytes = 0;                   ///< Data size processed
    bool success = true;                           ///< Operation success flag
    std::string error_code;                        ///< Error code if applicable
    std::string error_message;                     ///< Error message if applicable

    AuditEvent() {
        timestamp = std::chrono::system_clock::now();
        event_id = generateUniqueId();
        version = "2.0.0";
    }

private:
    std::string generateUniqueId() const;
};

/**
 * @brief Audit event query criteria
 */
struct AuditQuery {
    // Time range
    std::chrono::system_clock::time_point start_time;
    std::chrono::system_clock::time_point end_time;

    // Event filtering
    std::vector<AuditEventType> event_types;       ///< Event types to include
    std::vector<std::string> user_ids;             ///< User IDs to include
    std::vector<std::string> session_ids;          ///< Session IDs to include
    std::vector<std::string> transaction_ids;      ///< Transaction IDs to include

    // Content filtering
    std::map<std::string, std::string> metadata_filters; ///< Metadata filters
    std::string text_search;                       ///< Text search query
    bool include_successful_events = true;         ///< Include successful events
    bool include_error_events = true;              ///< Include error events

    // Result options
    size_t max_results = 10000;                   ///< Maximum results to return
    size_t offset = 0;                            ///< Result offset
    std::string sort_by = "timestamp";             ///< Sort field
    bool sort_descending = true;                   ///< Sort order

    AuditQuery() {
        start_time = std::chrono::system_clock::now() - std::chrono::hours(24);
        end_time = std::chrono::system_clock::now();
    }
};

/**
 * @brief Secure Telemetry Logger for Banking ML Compliance
 *
 * Professional audit logging system providing comprehensive compliance
 * and security monitoring for banking-grade machine learning operations.
 *
 * Key Features:
 * - Comprehensive audit trail for all ML operations
 * - Banking compliance (PCI DSS, GDPR, SOX, HIPAA)
 * - Encrypted storage with digital signatures
 * - Real-time monitoring and anomaly detection
 * - Structured logging with JSON format
 * - Automatic log rotation and retention management
 * - Performance monitoring and alerting
 * - Privacy-preserving logging with data anonymization
 * - Thread-safe asynchronous logging
 * - Integration with external SIEM systems
 *
 * Compliance Standards:
 * - PCI DSS Level 1 compliance for payment data
 * - GDPR compliance for personal data protection
 * - SOX compliance for financial reporting
 * - ISO 27001 information security management
 * - NIST Cybersecurity Framework alignment
 */
class SecureTelemetryLogger {
public:
    /**
     * @brief Constructor with default configuration
     */
    SecureTelemetryLogger();

    /**
     * @brief Constructor with custom configuration
     * @param config Secure telemetry configuration
     */
    explicit SecureTelemetryLogger(const SecureTelemetryConfig& config);

    /**
     * @brief Destructor
     */
    ~SecureTelemetryLogger();

    /**
     * @brief Initialize secure telemetry logger
     * @param ml_config ML configuration for compliance settings
     * @return True if initialization successful
     */
    bool initialize(const AdvancedMLConfig& ml_config);

    /**
     * @brief Check if logger is initialized and ready
     * @return True if ready for logging operations
     */
    bool isInitialized() const;

    /**
     * @brief Log ML processing event with full audit trail
     * @param event_type Type of ML event
     * @param request_data Request data summary
     * @param result_data Result data summary
     * @param user_id Associated user ID
     * @return True if event logged successfully
     */
    bool logMLEvent(const std::string& event_type,
                    const std::map<std::string, std::string>& request_data,
                    const std::map<std::string, std::string>& result_data,
                    const std::string& user_id = "");

    /**
     * @brief Log comprehensive audit event
     * @param event Audit event to log
     * @return True if event logged successfully
     */
    bool logAuditEvent(const AuditEvent& event);

    /**
     * @brief Log security violation event
     * @param violation_type Type of security violation
     * @param violation_details Violation details
     * @param user_context User context information
     * @param severity Violation severity (LOW, MEDIUM, HIGH, CRITICAL)
     * @return True if violation logged successfully
     */
    bool logSecurityViolation(const std::string& violation_type,
                             const std::map<std::string, std::string>& violation_details,
                             const std::map<std::string, std::string>& user_context,
                             const std::string& severity = "MEDIUM");

    /**
     * @brief Log compliance check event
     * @param compliance_type Type of compliance check
     * @param check_results Compliance check results
     * @param compliance_status Status (PASS, FAIL, WARNING)
     * @param remediation_actions Required remediation actions
     * @return True if compliance event logged successfully
     */
    bool logComplianceEvent(const std::string& compliance_type,
                           const std::map<std::string, std::string>& check_results,
                           const std::string& compliance_status,
                           const std::vector<std::string>& remediation_actions = {});

    /**
     * @brief Log authentication/authorization event
     * @param auth_type Authentication type (LOGIN, LOGOUT, ACCESS_GRANT, ACCESS_DENY)
     * @param user_id User identifier
     * @param resource_accessed Resource being accessed
     * @param auth_result Authentication result
     * @param auth_context Authentication context
     * @return True if auth event logged successfully
     */
    bool logAuthenticationEvent(const std::string& auth_type,
                               const std::string& user_id,
                               const std::string& resource_accessed,
                               bool auth_result,
                               const std::map<std::string, std::string>& auth_context = {});

    /**
     * @brief Log performance monitoring event
     * @param component_name Component being monitored
     * @param performance_metrics Performance metrics
     * @param alert_level Alert level (INFO, WARNING, CRITICAL)
     * @param threshold_violations Threshold violations detected
     * @return True if performance event logged successfully
     */
    bool logPerformanceEvent(const std::string& component_name,
                            const std::map<std::string, float>& performance_metrics,
                            const std::string& alert_level = "INFO",
                            const std::vector<std::string>& threshold_violations = {});

    /**
     * @brief Query audit events based on criteria
     * @param query Query criteria
     * @param results Output audit events matching criteria
     * @return Number of events found
     */
    size_t queryAuditEvents(const AuditQuery& query,
                           std::vector<AuditEvent>& results);

    /**
     * @brief Export audit trail to external format
     * @param output_path Path to export audit data
     * @param format Export format ("json", "csv", "xml", "siem")
     * @param query Query criteria for export
     * @param include_metadata Include metadata in export
     * @return True if export successful
     */
    bool exportAuditTrail(const std::string& output_path,
                         const std::string& format = "json",
                         const AuditQuery& query = AuditQuery{},
                         bool include_metadata = true);

    /**
     * @brief Generate compliance report
     * @param compliance_type Compliance type ("PCI_DSS", "GDPR", "SOX", "HIPAA")
     * @param report_period Report period in hours
     * @param output_path Output path for report
     * @return True if report generated successfully
     */
    bool generateComplianceReport(const std::string& compliance_type,
                                 int report_period_hours = 24,
                                 const std::string& output_path = "");

    /**
     * @brief Verify audit log integrity
     * @param start_time Start time for verification
     * @param end_time End time for verification
     * @param integrity_report Output integrity verification report
     * @return True if audit logs are intact
     */
    bool verifyAuditIntegrity(const std::chrono::system_clock::time_point& start_time,
                             const std::chrono::system_clock::time_point& end_time,
                             std::map<std::string, std::string>& integrity_report);

    /**
     * @brief Set telemetry configuration
     * @param config New configuration
     * @return True if configuration applied successfully
     */
    bool setConfiguration(const SecureTelemetryConfig& config);

    /**
     * @brief Get current telemetry configuration
     * @return Current configuration
     */
    SecureTelemetryConfig getConfiguration() const;

    /**
     * @brief Enable/disable specific logging features
     * @param enable_audit Enable audit logging
     * @param enable_security Enable security logging
     * @param enable_compliance Enable compliance logging
     * @param enable_performance Enable performance logging
     */
    void setLoggingFeatures(bool enable_audit,
                           bool enable_security,
                           bool enable_compliance,
                           bool enable_performance);

    /**
     * @brief Set compliance mode for logging
     * @param compliance_level Compliance level ("PCI_DSS_L1", "GDPR", "SOX")
     * @param anonymize_data Anonymize personal data
     * @param encryption_required Require encryption
     */
    void setComplianceMode(const std::string& compliance_level,
                          bool anonymize_data = true,
                          bool encryption_required = true);

    /**
     * @brief Get logging statistics
     * @param total_events Total events logged
     * @param successful_events Successful events
     * @param failed_events Failed events
     * @param avg_log_time Average logging time (ms)
     * @param disk_usage_mb Disk usage (MB)
     */
    void getLoggingStatistics(size_t& total_events,
                             size_t& successful_events,
                             size_t& failed_events,
                             double& avg_log_time,
                             float& disk_usage_mb) const;

    /**
     * @brief Flush pending log events to storage
     * @param force_flush Force immediate flush
     * @return True if flush successful
     */
    bool flushLogs(bool force_flush = false);

    /**
     * @brief Perform log maintenance (rotation, cleanup, compression)
     * @return True if maintenance successful
     */
    bool performLogMaintenance();

    /**
     * @brief Get last logging error
     * @return Human-readable error message
     */
    std::string getLastError() const;

    /**
     * @brief Validate logging system integrity
     * @param integrity_report Output integrity validation report
     * @return True if logging system is functioning correctly
     */
    bool validateSystemIntegrity(std::map<std::string, std::string>& integrity_report);

private:
    // Configuration and state
    SecureTelemetryConfig config_;
    mutable std::mutex config_mutex_;
    std::atomic<bool> initialized_{false};
    std::atomic<bool> shutdown_requested_{false};

    // Asynchronous logging
    std::mutex log_queue_mutex_;
    std::condition_variable log_queue_cv_;
    std::queue<AuditEvent> log_queue_;
    std::vector<std::thread> worker_threads_;

    // File management
    std::mutex file_mutex_;
    std::unique_ptr<std::ofstream> current_log_file_;
    std::string current_log_filename_;
    size_t current_file_size_;
    std::chrono::system_clock::time_point last_rotation_time_;

    // Statistics
    mutable std::mutex stats_mutex_;
    std::atomic<size_t> total_events_{0};
    std::atomic<size_t> successful_events_{0};
    std::atomic<size_t> failed_events_{0};
    std::atomic<double> total_log_time_{0.0};

    // Security and encryption
    std::string encryption_key_;
    std::unique_ptr<class EncryptionManager> encryption_manager_;
    std::unique_ptr<class IntegrityVerifier> integrity_verifier_;

    // Monitoring and anomaly detection
    std::unique_ptr<class AnomalyDetector> anomaly_detector_;
    std::unique_ptr<class AlertManager> alert_manager_;

    // Error handling
    mutable std::mutex error_mutex_;
    std::string last_error_;
    std::vector<std::string> error_history_;

    /**
     * @brief Initialize logging components
     * @return True if initialization successful
     */
    bool initializeComponents();

    /**
     * @brief Initialize secure storage
     * @return True if storage initialization successful
     */
    bool initializeSecureStorage();

    /**
     * @brief Worker thread function for asynchronous logging
     */
    void logWorkerThread();

    /**
     * @brief Write audit event to secure storage
     * @param event Audit event to write
     * @return True if write successful
     */
    bool writeAuditEvent(const AuditEvent& event);

    /**
     * @brief Encrypt audit event data
     * @param event_data Plain event data
     * @param encrypted_data Output encrypted data
     * @return True if encryption successful
     */
    bool encryptEventData(const std::string& event_data,
                         std::string& encrypted_data);

    /**
     * @brief Generate digital signature for event
     * @param event_data Event data to sign
     * @param signature Output digital signature
     * @return True if signature generation successful
     */
    bool generateEventSignature(const std::string& event_data,
                               std::string& signature);

    /**
     * @brief Anonymize personal data in event
     * @param event Event to anonymize
     */
    void anonymizePersonalData(AuditEvent& event);

    /**
     * @brief Check if log rotation is needed
     * @return True if rotation needed
     */
    bool isLogRotationNeeded();

    /**
     * @brief Perform log file rotation
     * @return True if rotation successful
     */
    bool rotateLogFile();

    /**
     * @brief Generate unique event identifier
     * @return Unique event ID
     */
    std::string generateEventId();

    /**
     * @brief Get current log filename
     * @return Current log filename
     */
    std::string getCurrentLogFilename();

    /**
     * @brief Update logging statistics
     * @param log_time Logging time
     * @param success Whether logging was successful
     */
    void updateStatistics(double log_time, bool success);

    /**
     * @brief Set last error message
     * @param error_message Error message to set
     */
    void setLastError(const std::string& error_message);

    // Disable copy constructor and assignment
    SecureTelemetryLogger(const SecureTelemetryLogger&) = delete;
    SecureTelemetryLogger& operator=(const SecureTelemetryLogger&) = delete;

    // Enable move semantics
    SecureTelemetryLogger(SecureTelemetryLogger&&) noexcept;
    SecureTelemetryLogger& operator=(SecureTelemetryLogger&&) noexcept;
};

/**
 * @brief Secure Telemetry Logger utility functions
 */
class SecureTelemetryUtils {
public:
    /**
     * @brief Validate secure telemetry configuration
     * @param config Configuration to validate
     * @param validation_errors Output validation errors
     * @return True if configuration is valid
     */
    static bool validateConfiguration(const SecureTelemetryConfig& config,
                                     std::vector<std::string>& validation_errors);

    /**
     * @brief Generate audit event summary report
     * @param events Vector of audit events
     * @param include_statistics Include statistical analysis
     * @return Formatted audit summary report
     */
    static std::string generateAuditSummary(const std::vector<AuditEvent>& events,
                                           bool include_statistics = true);

    /**
     * @brief Convert audit event to JSON format
     * @param event Audit event to convert
     * @param include_metadata Include metadata in JSON
     * @return JSON representation of event
     */
    static std::string auditEventToJson(const AuditEvent& event,
                                       bool include_metadata = true);

    /**
     * @brief Parse audit event from JSON format
     * @param json_data JSON data to parse
     * @param event Output audit event
     * @return True if parsing successful
     */
    static bool auditEventFromJson(const std::string& json_data,
                                  AuditEvent& event);

    /**
     * @brief Estimate storage requirements for audit logging
     * @param config Logging configuration
     * @param expected_events_per_day Expected events per day
     * @return Storage requirement estimates
     */
    static std::map<std::string, std::string> estimateStorageRequirements(
        const SecureTelemetryConfig& config,
        size_t expected_events_per_day);
};

} // namespace face
} // namespace unlook