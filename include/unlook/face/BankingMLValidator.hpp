#pragma once

/**
 * @file BankingMLValidator.hpp
 * @brief Banking ML Validator for Compliance Validation and Risk Assessment
 *
 * Professional banking compliance validator providing comprehensive validation
 * of machine learning operations for banking-grade biometric authentication
 * with regulatory compliance, risk assessment, and audit capabilities.
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

namespace unlook {
namespace face {

/**
 * @brief Banking compliance standards
 */
enum class BankingComplianceStandard {
    PCI_DSS_L1,        ///< PCI DSS Level 1 compliance
    PCI_DSS_L2,        ///< PCI DSS Level 2 compliance
    GDPR,              ///< GDPR compliance
    SOX,               ///< Sarbanes-Oxley compliance
    HIPAA,             ///< HIPAA compliance (healthcare)
    BASEL_III,         ///< Basel III compliance
    FFIEC,             ///< FFIEC guidance compliance
    ISO_27001,         ///< ISO 27001 information security
    NIST_CSF,          ///< NIST Cybersecurity Framework
    CUSTOM             ///< Custom compliance requirements
};

/**
 * @brief Banking ML validator configuration
 */
struct BankingMLValidatorConfig {
    // Core compliance settings
    BankingComplianceStandard primary_standard = BankingComplianceStandard::PCI_DSS_L1;
    std::vector<BankingComplianceStandard> additional_standards;
    std::string compliance_level = "HIGH";       ///< LOW, MEDIUM, HIGH, CRITICAL
    bool strict_mode = true;                     ///< Enable strict compliance checking

    // Risk assessment settings
    bool enable_risk_assessment = true;          ///< Enable risk assessment
    float max_acceptable_far = 0.0001f;          ///< Maximum false accept rate (0.01%)
    float max_acceptable_frr = 0.03f;            ///< Maximum false reject rate (3%)
    float risk_threshold = 0.1f;                 ///< Risk threshold (0-1)
    bool enable_continuous_monitoring = true;    ///< Enable continuous risk monitoring

    // Performance validation settings
    bool validate_response_times = true;         ///< Validate ML response times
    int max_response_time_ms = 5000;            ///< Maximum response time (5 seconds)
    bool validate_accuracy_metrics = true;       ///< Validate accuracy metrics
    float min_accuracy_threshold = 0.95f;        ///< Minimum accuracy threshold

    // Data protection settings
    bool validate_data_encryption = true;        ///< Validate data encryption
    bool validate_data_integrity = true;         ///< Validate data integrity
    bool validate_access_controls = true;        ///< Validate access controls
    bool validate_audit_trails = true;           ///< Validate audit trails

    // Regulatory compliance settings
    bool enable_gdpr_validation = true;          ///< Enable GDPR validation
    bool enable_sox_validation = false;          ///< Enable SOX validation
    bool enable_pci_validation = true;           ///< Enable PCI DSS validation
    bool validate_data_residency = true;         ///< Validate data residency

    // Alert and reporting settings
    bool enable_compliance_alerts = true;        ///< Enable compliance alerts
    bool enable_violation_reporting = true;      ///< Enable violation reporting
    bool generate_compliance_reports = true;     ///< Generate compliance reports
    int report_generation_interval_hours = 24;   ///< Report generation interval

    bool validate() const;
    std::string toString() const;
};

/**
 * @brief Banking compliance validation result
 */
struct BankingComplianceResult {
    // Overall compliance status
    bool is_compliant = false;                   ///< Overall compliance status
    std::string compliance_level_achieved;       ///< Achieved compliance level
    float compliance_score = 0.0f;              ///< Compliance score (0-1)

    // Compliance standard results
    std::map<BankingComplianceStandard, bool> standard_compliance;
    std::map<std::string, float> compliance_metrics;

    // Risk assessment results
    float overall_risk_score = 1.0f;            ///< Overall risk score (0-1, lower is better)
    float far_estimate = 1.0f;                  ///< False accept rate estimate
    float frr_estimate = 1.0f;                  ///< False reject rate estimate
    std::string risk_level;                     ///< LOW, MEDIUM, HIGH, CRITICAL

    // Validation details
    struct ValidationDetails {
        bool data_encryption_valid = false;      ///< Data encryption validation
        bool data_integrity_valid = false;       ///< Data integrity validation
        bool access_controls_valid = false;      ///< Access controls validation
        bool audit_trail_valid = false;          ///< Audit trail validation
        bool performance_valid = false;          ///< Performance validation
        bool accuracy_valid = false;             ///< Accuracy validation

        std::vector<std::string> passed_checks;  ///< Passed validation checks
        std::vector<std::string> failed_checks;  ///< Failed validation checks
        std::vector<std::string> warnings;       ///< Validation warnings
    } validation_details;

    // Remediation information
    std::vector<std::string> required_actions;   ///< Required remediation actions
    std::vector<std::string> recommended_actions; ///< Recommended improvements
    std::string remediation_priority;            ///< IMMEDIATE, HIGH, MEDIUM, LOW

    // Timestamps and metadata
    std::chrono::system_clock::time_point validation_timestamp;
    std::string validator_version;
    std::map<std::string, std::string> metadata;

    BankingComplianceResult() {
        validation_timestamp = std::chrono::system_clock::now();
        validator_version = "2.0.0";
    }
};

/**
 * @brief Banking transaction context for validation
 */
struct BankingTransactionContext {
    std::string transaction_id;                  ///< Unique transaction identifier
    std::string transaction_type;                ///< Transaction type
    float transaction_amount = 0.0f;             ///< Transaction amount
    std::string currency_code = "USD";           ///< Currency code
    std::string merchant_category;               ///< Merchant category
    std::string payment_method;                  ///< Payment method
    std::string risk_category = "MEDIUM";        ///< Risk category
    bool high_value_transaction = false;         ///< High value flag
    std::map<std::string, std::string> additional_context;

    BankingTransactionContext() = default;
};

/**
 * @brief Banking ML Validator for Compliance Validation and Risk Assessment
 *
 * Professional banking compliance validator ensuring all machine learning
 * operations meet strict banking and financial industry requirements.
 *
 * Key Features:
 * - Comprehensive banking compliance validation (PCI DSS, GDPR, SOX, Basel III)
 * - Real-time risk assessment and monitoring
 * - Performance validation with SLA compliance
 * - Data protection and privacy validation
 * - Regulatory compliance reporting
 * - Continuous monitoring and alerting
 * - Audit trail validation and integrity checking
 * - Risk-based authentication controls
 * - Multi-standard compliance support
 * - Automated remediation recommendations
 *
 * Banking Standards Supported:
 * - PCI DSS Level 1/2 for payment card industry
 * - GDPR for data protection and privacy
 * - SOX for financial reporting controls
 * - Basel III for banking risk management
 * - FFIEC guidance for financial institutions
 * - ISO 27001 for information security
 * - NIST Cybersecurity Framework
 * - Custom compliance frameworks
 */
class BankingMLValidator {
public:
    /**
     * @brief Constructor with default configuration
     */
    BankingMLValidator();

    /**
     * @brief Constructor with custom configuration
     * @param config Banking ML validator configuration
     */
    explicit BankingMLValidator(const BankingMLValidatorConfig& config);

    /**
     * @brief Destructor
     */
    ~BankingMLValidator();

    /**
     * @brief Initialize banking ML validator
     * @param ml_config ML configuration for compliance settings
     * @return True if initialization successful
     */
    bool initialize(const AdvancedMLConfig& ml_config);

    /**
     * @brief Check if validator is initialized and ready
     * @return True if ready for validation operations
     */
    bool isInitialized() const;

    /**
     * @brief Validate banking compliance for ML operation
     * @param ml_result ML processing result to validate
     * @param compliance_result Output compliance validation result
     * @return True if ML result meets banking compliance requirements
     */
    bool validateBankingCompliance(const AdvancedMLResult& ml_result,
                                  BankingComplianceResult& compliance_result);

    /**
     * @brief Validate banking compliance with transaction context
     * @param ml_result ML processing result
     * @param transaction_context Banking transaction context
     * @param compliance_result Output compliance validation result
     * @return True if compliant with transaction-specific requirements
     */
    bool validateTransactionCompliance(const AdvancedMLResult& ml_result,
                                      const BankingTransactionContext& transaction_context,
                                      BankingComplianceResult& compliance_result);

    /**
     * @brief Validate banking context information
     * @param banking_context Banking context to validate
     * @return True if banking context is valid
     */
    bool validateBankingContext(const std::map<std::string, std::string>& banking_context);

    /**
     * @brief Assess risk for banking ML operation
     * @param ml_result ML processing result
     * @param transaction_context Transaction context
     * @param risk_score Output risk score (0-1, lower is better)
     * @param risk_factors Output identified risk factors
     * @return Risk level (LOW, MEDIUM, HIGH, CRITICAL)
     */
    std::string assessRisk(const AdvancedMLResult& ml_result,
                          const BankingTransactionContext& transaction_context,
                          float& risk_score,
                          std::vector<std::string>& risk_factors);

    /**
     * @brief Validate performance metrics against banking SLAs
     * @param performance_metrics Performance metrics to validate
     * @param sla_requirements SLA requirements
     * @param validation_result Output validation result
     * @return True if performance meets banking SLAs
     */
    bool validatePerformanceMetrics(const std::map<std::string, float>& performance_metrics,
                                   const std::map<std::string, float>& sla_requirements,
                                   std::map<std::string, std::string>& validation_result);

    /**
     * @brief Validate data protection and privacy requirements
     * @param ml_result ML processing result
     * @param data_context Data processing context
     * @param protection_result Output data protection validation result
     * @return True if data protection requirements are met
     */
    bool validateDataProtection(const AdvancedMLResult& ml_result,
                               const std::map<std::string, std::string>& data_context,
                               std::map<std::string, std::string>& protection_result);

    /**
     * @brief Generate comprehensive compliance report
     * @param compliance_results Vector of compliance results
     * @param report_period_hours Report period in hours
     * @param output_path Output path for report
     * @param report_format Report format ("json", "pdf", "html")
     * @return True if report generated successfully
     */
    bool generateComplianceReport(const std::vector<BankingComplianceResult>& compliance_results,
                                 int report_period_hours = 24,
                                 const std::string& output_path = "",
                                 const std::string& report_format = "json");

    /**
     * @brief Validate regulatory compliance requirements
     * @param compliance_standard Compliance standard to validate
     * @param ml_result ML processing result
     * @param regulatory_context Regulatory context
     * @param compliance_details Output compliance details
     * @return True if regulatory requirements are met
     */
    bool validateRegulatoryCompliance(BankingComplianceStandard compliance_standard,
                                     const AdvancedMLResult& ml_result,
                                     const std::map<std::string, std::string>& regulatory_context,
                                     std::map<std::string, std::string>& compliance_details);

    /**
     * @brief Monitor continuous compliance for ongoing operations
     * @param ml_results Vector of recent ML results
     * @param monitoring_result Output monitoring result
     * @return True if continuous compliance is maintained
     */
    bool monitorContinuousCompliance(const std::vector<AdvancedMLResult>& ml_results,
                                    BankingComplianceResult& monitoring_result);

    /**
     * @brief Set banking validator configuration
     * @param config New validator configuration
     * @return True if configuration applied successfully
     */
    bool setConfiguration(const BankingMLValidatorConfig& config);

    /**
     * @brief Get current validator configuration
     * @return Current validator configuration
     */
    BankingMLValidatorConfig getConfiguration() const;

    /**
     * @brief Set compliance level and standards
     * @param compliance_level Compliance level ("HIGH", "CRITICAL")
     * @param audit_level Audit logging level
     */
    void setComplianceLevel(const std::string& compliance_level,
                           const std::string& audit_level = "COMPREHENSIVE");

    /**
     * @brief Enable/disable specific compliance validations
     * @param enable_risk_assessment Enable risk assessment
     * @param enable_performance_validation Enable performance validation
     * @param enable_data_protection Enable data protection validation
     * @param enable_regulatory_compliance Enable regulatory compliance
     */
    void setValidationFeatures(bool enable_risk_assessment,
                              bool enable_performance_validation,
                              bool enable_data_protection,
                              bool enable_regulatory_compliance);

    /**
     * @brief Get banking validation statistics
     * @param total_validations Total validations performed
     * @param compliant_validations Compliant validations
     * @param non_compliant_validations Non-compliant validations
     * @param avg_compliance_score Average compliance score
     * @param avg_risk_score Average risk score
     */
    void getValidationStatistics(size_t& total_validations,
                                size_t& compliant_validations,
                                size_t& non_compliant_validations,
                                float& avg_compliance_score,
                                float& avg_risk_score) const;

    /**
     * @brief Reset validation statistics
     */
    void resetStatistics();

    /**
     * @brief Get last validation error
     * @return Human-readable error message
     */
    std::string getLastError() const;

    /**
     * @brief Validate banking ML validator system integrity
     * @param integrity_report Output integrity validation report
     * @return True if validator system is functioning correctly
     */
    bool validateSystemIntegrity(std::map<std::string, std::string>& integrity_report);

private:
    // Configuration and state
    BankingMLValidatorConfig config_;
    mutable std::mutex config_mutex_;
    std::atomic<bool> initialized_{false};
    std::atomic<bool> strict_mode_enabled_{true};

    // Compliance standards and thresholds
    std::map<BankingComplianceStandard, std::map<std::string, float>> compliance_thresholds_;
    std::map<std::string, float> performance_sla_thresholds_;

    // Statistics tracking
    mutable std::mutex stats_mutex_;
    std::atomic<size_t> total_validations_{0};
    std::atomic<size_t> compliant_validations_{0};
    std::atomic<size_t> non_compliant_validations_{0};
    std::atomic<float> cumulative_compliance_score_{0.0f};
    std::atomic<float> cumulative_risk_score_{0.0f};

    // Monitoring and alerting
    std::mutex alert_mutex_;
    std::vector<std::string> recent_violations_;
    std::chrono::system_clock::time_point last_report_time_;

    // Error handling
    mutable std::mutex error_mutex_;
    std::string last_error_;
    std::vector<std::string> error_history_;

    /**
     * @brief Initialize banking compliance components
     * @return True if initialization successful
     */
    bool initializeComplianceComponents();

    /**
     * @brief Initialize compliance thresholds for different standards
     */
    void initializeComplianceThresholds();

    /**
     * @brief Validate PCI DSS compliance requirements
     * @param ml_result ML processing result
     * @param compliance_details Output compliance details
     * @return True if PCI DSS compliant
     */
    bool validatePCIDSSCompliance(const AdvancedMLResult& ml_result,
                                 std::map<std::string, std::string>& compliance_details);

    /**
     * @brief Validate GDPR compliance requirements
     * @param ml_result ML processing result
     * @param compliance_details Output compliance details
     * @return True if GDPR compliant
     */
    bool validateGDPRCompliance(const AdvancedMLResult& ml_result,
                               std::map<std::string, std::string>& compliance_details);

    /**
     * @brief Validate SOX compliance requirements
     * @param ml_result ML processing result
     * @param compliance_details Output compliance details
     * @return True if SOX compliant
     */
    bool validateSOXCompliance(const AdvancedMLResult& ml_result,
                              std::map<std::string, std::string>& compliance_details);

    /**
     * @brief Calculate comprehensive risk score
     * @param ml_result ML processing result
     * @param transaction_context Transaction context
     * @return Risk score (0-1, lower is better)
     */
    float calculateRiskScore(const AdvancedMLResult& ml_result,
                            const BankingTransactionContext& transaction_context);

    /**
     * @brief Identify risk factors for ML operation
     * @param ml_result ML processing result
     * @param transaction_context Transaction context
     * @param risk_factors Output identified risk factors
     * @return Number of risk factors identified
     */
    size_t identifyRiskFactors(const AdvancedMLResult& ml_result,
                              const BankingTransactionContext& transaction_context,
                              std::vector<std::string>& risk_factors);

    /**
     * @brief Generate remediation recommendations
     * @param compliance_result Compliance validation result
     * @param remediation_actions Output remediation actions
     * @param priority Output priority level
     */
    void generateRemediationRecommendations(const BankingComplianceResult& compliance_result,
                                           std::vector<std::string>& remediation_actions,
                                           std::string& priority);

    /**
     * @brief Update validation statistics
     * @param compliance_score Compliance score
     * @param risk_score Risk score
     * @param is_compliant Whether validation passed
     */
    void updateStatistics(float compliance_score, float risk_score, bool is_compliant);

    /**
     * @brief Send compliance alert for violations
     * @param violation_type Type of violation
     * @param severity Violation severity
     * @param details Violation details
     */
    void sendComplianceAlert(const std::string& violation_type,
                            const std::string& severity,
                            const std::map<std::string, std::string>& details);

    /**
     * @brief Set last error message
     * @param error_message Error message to set
     */
    void setLastError(const std::string& error_message);

    // Disable copy constructor and assignment
    BankingMLValidator(const BankingMLValidator&) = delete;
    BankingMLValidator& operator=(const BankingMLValidator&) = delete;

    // Enable move semantics
    BankingMLValidator(BankingMLValidator&&) noexcept;
    BankingMLValidator& operator=(BankingMLValidator&&) noexcept;
};

/**
 * @brief Banking ML Validator utility functions
 */
class BankingMLValidatorUtils {
public:
    /**
     * @brief Validate banking ML validator configuration
     * @param config Configuration to validate
     * @param validation_errors Output validation errors
     * @return True if configuration is valid
     */
    static bool validateConfiguration(const BankingMLValidatorConfig& config,
                                     std::vector<std::string>& validation_errors);

    /**
     * @brief Generate banking compliance summary report
     * @param compliance_results Vector of compliance results
     * @param include_recommendations Include remediation recommendations
     * @return Formatted compliance summary report
     */
    static std::string generateComplianceSummary(
        const std::vector<BankingComplianceResult>& compliance_results,
        bool include_recommendations = true);

    /**
     * @brief Convert compliance standard to string
     * @param standard Compliance standard
     * @return String representation of standard
     */
    static std::string complianceStandardToString(BankingComplianceStandard standard);

    /**
     * @brief Parse compliance standard from string
     * @param standard_str String representation
     * @return Compliance standard
     */
    static BankingComplianceStandard complianceStandardFromString(const std::string& standard_str);

    /**
     * @brief Calculate compliance score aggregate
     * @param individual_scores Vector of individual compliance scores
     * @param weights Optional weights for each score
     * @return Aggregate compliance score
     */
    static float calculateAggregateComplianceScore(
        const std::vector<float>& individual_scores,
        const std::vector<float>& weights = {});
};

} // namespace face
} // namespace unlook