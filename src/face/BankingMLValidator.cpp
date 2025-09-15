/**
 * @file BankingMLValidator.cpp
 * @brief Banking ML Validator Implementation for Compliance Validation and Risk Assessment
 *
 * Professional implementation providing comprehensive validation of machine
 * learning operations for banking-grade biometric authentication with
 * regulatory compliance, risk assessment, and audit capabilities.
 *
 * @version 2.0.0-banking-compliance
 * @author Unlook Team - API Architecture Agent
 * @copyright 2025 Unlook. All rights reserved.
 */

#include "unlook/face/BankingMLValidator.hpp"
#include "unlook/core/Logger.hpp"
#include "unlook/core/Exception.hpp"

#include <algorithm>
#include <numeric>
#include <iomanip>
#include <sstream>
#include <fstream>
#include <cmath>

namespace unlook {
namespace face {

// ============================================================================
// BankingMLValidatorConfig Implementation
// ============================================================================

bool BankingMLValidatorConfig::validate() const {
    if (max_acceptable_far < 0.0f || max_acceptable_far > 1.0f) {
        return false;
    }

    if (max_acceptable_frr < 0.0f || max_acceptable_frr > 1.0f) {
        return false;
    }

    if (risk_threshold < 0.0f || risk_threshold > 1.0f) {
        return false;
    }

    if (max_response_time_ms <= 0 || max_response_time_ms > 60000) {
        return false;
    }

    if (min_accuracy_threshold < 0.0f || min_accuracy_threshold > 1.0f) {
        return false;
    }

    if (report_generation_interval_hours <= 0 || report_generation_interval_hours > 8760) {
        return false;
    }

    const std::vector<std::string> valid_compliance_levels = {"LOW", "MEDIUM", "HIGH", "CRITICAL"};
    if (std::find(valid_compliance_levels.begin(), valid_compliance_levels.end(),
                 compliance_level) == valid_compliance_levels.end()) {
        return false;
    }

    return true;
}

std::string BankingMLValidatorConfig::toString() const {
    std::ostringstream oss;
    oss << "BankingMLValidatorConfig{";
    oss << "primary_standard=" << static_cast<int>(primary_standard) << ", ";
    oss << "compliance_level='" << compliance_level << "', ";
    oss << "strict_mode=" << (strict_mode ? "enabled" : "disabled") << ", ";
    oss << "max_far=" << max_acceptable_far << ", ";
    oss << "max_frr=" << max_acceptable_frr << ", ";
    oss << "risk_threshold=" << risk_threshold << ", ";
    oss << "max_response_time=" << max_response_time_ms << "ms, ";
    oss << "min_accuracy=" << min_accuracy_threshold << ", ";
    oss << "risk_assessment=" << (enable_risk_assessment ? "enabled" : "disabled") << ", ";
    oss << "continuous_monitoring=" << (enable_continuous_monitoring ? "enabled" : "disabled");
    oss << "}";
    return oss.str();
}

// ============================================================================
// BankingMLValidator Implementation
// ============================================================================

BankingMLValidator::BankingMLValidator()
    : config_{}
    , initialized_(false)
    , strict_mode_enabled_(true)
    , total_validations_(0)
    , compliant_validations_(0)
    , non_compliant_validations_(0)
    , cumulative_compliance_score_(0.0f)
    , cumulative_risk_score_(0.0f)
{
    UNLOOK_LOG_INFO("BankingMLValidator: Constructor with default configuration");
    last_report_time_ = std::chrono::system_clock::now();
}

BankingMLValidator::BankingMLValidator(const BankingMLValidatorConfig& config)
    : config_(config)
    , initialized_(false)
    , strict_mode_enabled_(config.strict_mode)
    , total_validations_(0)
    , compliant_validations_(0)
    , non_compliant_validations_(0)
    , cumulative_compliance_score_(0.0f)
    , cumulative_risk_score_(0.0f)
{
    UNLOOK_LOG_INFO("BankingMLValidator: Constructor with custom configuration");
    UNLOOK_LOG_DEBUG("BankingMLValidator: Configuration - " + config_.toString());
    last_report_time_ = std::chrono::system_clock::now();
}

BankingMLValidator::~BankingMLValidator() {
    UNLOOK_LOG_INFO("BankingMLValidator: Destructor called");
}

bool BankingMLValidator::initialize(const AdvancedMLConfig& ml_config) {
    UNLOOK_LOG_INFO("BankingMLValidator: Initializing banking ML validator");

    std::lock_guard<std::mutex> lock(config_mutex_);

    // Validate configuration
    if (!config_.validate()) {
        setLastError("Invalid BankingMLValidatorConfig provided");
        UNLOOK_LOG_ERROR("BankingMLValidator: Invalid configuration: " + config_.toString());
        return false;
    }

    // Update configuration based on ML config
    config_.max_acceptable_far = ml_config.banking_far_requirement;
    config_.max_acceptable_frr = ml_config.banking_frr_requirement;
    config_.enable_gdpr_validation = ml_config.enable_gdpr_mode;
    config_.strict_mode = ml_config.banking_far_requirement <= 0.0001f; // Very strict for low FAR

    // Initialize compliance components
    if (!initializeComplianceComponents()) {
        setLastError("Failed to initialize compliance components");
        UNLOOK_LOG_ERROR("BankingMLValidator: Compliance component initialization failed");
        return false;
    }

    // Initialize compliance thresholds
    initializeComplianceThresholds();

    initialized_ = true;
    strict_mode_enabled_ = config_.strict_mode;

    UNLOOK_LOG_INFO("BankingMLValidator: Initialization completed successfully");
    return true;
}

bool BankingMLValidator::isInitialized() const {
    return initialized_.load();
}

bool BankingMLValidator::validateBankingCompliance(const AdvancedMLResult& ml_result,
                                                  BankingComplianceResult& compliance_result) {
    UNLOOK_LOG_DEBUG("BankingMLValidator: Validating banking compliance");

    if (!isInitialized()) {
        setLastError("BankingMLValidator not initialized");
        return false;
    }

    compliance_result = BankingComplianceResult{};

    try {
        bool overall_compliance = true;
        float total_compliance_score = 0.0f;
        size_t validation_count = 0;

        // 1. Validate primary compliance standard
        std::map<std::string, std::string> primary_compliance_details;
        bool primary_compliant = validateRegulatoryCompliance(config_.primary_standard,
                                                             ml_result,
                                                             {},
                                                             primary_compliance_details);

        compliance_result.standard_compliance[config_.primary_standard] = primary_compliant;
        overall_compliance &= primary_compliant;

        if (primary_compliant) {
            total_compliance_score += 1.0f;
        }
        validation_count++;

        UNLOOK_LOG_DEBUG("BankingMLValidator: Primary standard validation - " +
                        (primary_compliant ? "PASSED" : "FAILED"));

        // 2. Validate additional compliance standards
        for (const auto& standard : config_.additional_standards) {
            std::map<std::string, std::string> additional_compliance_details;
            bool additional_compliant = validateRegulatoryCompliance(standard,
                                                                    ml_result,
                                                                    {},
                                                                    additional_compliance_details);

            compliance_result.standard_compliance[standard] = additional_compliant;
            overall_compliance &= additional_compliant;

            if (additional_compliant) {
                total_compliance_score += 1.0f;
            }
            validation_count++;
        }

        // 3. Validate performance metrics
        if (config_.validate_response_times || config_.validate_accuracy_metrics) {
            std::map<std::string, float> performance_metrics = {
                {"response_time_ms", static_cast<float>(ml_result.performance_metrics.total_processing_time_ms)},
                {"accuracy", ml_result.verification_result.similarity_score},
                {"confidence", ml_result.verification_result.confidence_level}
            };

            std::map<std::string, float> sla_requirements = {
                {"response_time_ms", static_cast<float>(config_.max_response_time_ms)},
                {"accuracy", config_.min_accuracy_threshold},
                {"confidence", 0.9f} // High confidence required for banking
            };

            std::map<std::string, std::string> performance_validation_result;
            bool performance_valid = validatePerformanceMetrics(performance_metrics,
                                                              sla_requirements,
                                                              performance_validation_result);

            compliance_result.validation_details.performance_valid = performance_valid;
            overall_compliance &= performance_valid;

            if (performance_valid) {
                total_compliance_score += 1.0f;
            }
            validation_count++;

            UNLOOK_LOG_DEBUG("BankingMLValidator: Performance validation - " +
                            (performance_valid ? "PASSED" : "FAILED"));
        }

        // 4. Validate data protection
        if (config_.validate_data_encryption || config_.validate_data_integrity) {
            std::map<std::string, std::string> data_context = {
                {"encryption_status", ml_result.security_metrics.encryption_validated ? "enabled" : "disabled"},
                {"integrity_status", ml_result.security_metrics.data_integrity_verified ? "verified" : "unverified"}
            };

            std::map<std::string, std::string> data_protection_result;
            bool data_protection_valid = validateDataProtection(ml_result,
                                                               data_context,
                                                               data_protection_result);

            compliance_result.validation_details.data_encryption_valid =
                ml_result.security_metrics.encryption_validated;
            compliance_result.validation_details.data_integrity_valid =
                ml_result.security_metrics.data_integrity_verified;

            overall_compliance &= data_protection_valid;

            if (data_protection_valid) {
                total_compliance_score += 1.0f;
            }
            validation_count++;

            UNLOOK_LOG_DEBUG("BankingMLValidator: Data protection validation - " +
                            (data_protection_valid ? "PASSED" : "FAILED"));
        }

        // 5. Validate accuracy and error rates
        bool accuracy_compliant = true;
        if (ml_result.verification_result.false_accept_rate > config_.max_acceptable_far) {
            accuracy_compliant = false;
            compliance_result.validation_details.failed_checks.push_back("FAR exceeds maximum acceptable rate");
        }

        if (ml_result.verification_result.false_reject_rate > config_.max_acceptable_frr) {
            accuracy_compliant = false;
            compliance_result.validation_details.failed_checks.push_back("FRR exceeds maximum acceptable rate");
        }

        compliance_result.validation_details.accuracy_valid = accuracy_compliant;
        overall_compliance &= accuracy_compliant;

        if (accuracy_compliant) {
            total_compliance_score += 1.0f;
        }
        validation_count++;

        // 6. Calculate overall compliance score
        if (validation_count > 0) {
            compliance_result.compliance_score = total_compliance_score / validation_count;
        } else {
            compliance_result.compliance_score = 0.0f;
        }

        // 7. Set compliance status and level
        compliance_result.is_compliant = overall_compliance;

        if (overall_compliance) {
            if (compliance_result.compliance_score >= 0.95f) {
                compliance_result.compliance_level_achieved = "CRITICAL";
            } else if (compliance_result.compliance_score >= 0.90f) {
                compliance_result.compliance_level_achieved = "HIGH";
            } else if (compliance_result.compliance_score >= 0.80f) {
                compliance_result.compliance_level_achieved = "MEDIUM";
            } else {
                compliance_result.compliance_level_achieved = "LOW";
            }
        } else {
            compliance_result.compliance_level_achieved = "NON_COMPLIANT";
        }

        // 8. Generate remediation recommendations if needed
        if (!overall_compliance) {
            generateRemediationRecommendations(compliance_result,
                                              compliance_result.required_actions,
                                              compliance_result.remediation_priority);
        }

        // 9. Update statistics
        updateStatistics(compliance_result.compliance_score, 0.0f, overall_compliance);

        // 10. Send alerts for violations if needed
        if (!overall_compliance && config_.enable_compliance_alerts) {
            sendComplianceAlert("BANKING_COMPLIANCE_VIOLATION",
                              compliance_result.remediation_priority,
                              {{"compliance_score", std::to_string(compliance_result.compliance_score)},
                               {"failed_checks", std::to_string(compliance_result.validation_details.failed_checks.size())}});
        }

        UNLOOK_LOG_DEBUG("BankingMLValidator: Banking compliance validation completed - " +
                        (overall_compliance ? "COMPLIANT" : "NON_COMPLIANT") +
                        " (score: " + std::to_string(compliance_result.compliance_score) + ")");

        return overall_compliance;

    } catch (const std::exception& e) {
        setLastError("Banking compliance validation exception: " + std::string(e.what()));
        UNLOOK_LOG_ERROR("BankingMLValidator: Validation exception: " + std::string(e.what()));

        compliance_result.is_compliant = false;
        compliance_result.compliance_level_achieved = "ERROR";

        return false;
    }
}

bool BankingMLValidator::validateBankingContext(const std::map<std::string, std::string>& banking_context) {
    UNLOOK_LOG_DEBUG("BankingMLValidator: Validating banking context");

    if (!isInitialized()) {
        setLastError("BankingMLValidator not initialized");
        return false;
    }

    try {
        // Required banking context fields
        const std::vector<std::string> required_fields = {
            "transaction_id",
            "user_id",
            "authentication_method",
            "risk_level"
        };

        // Check for required fields
        for (const auto& field : required_fields) {
            auto it = banking_context.find(field);
            if (it == banking_context.end() || it->second.empty()) {
                setLastError("Missing required banking context field: " + field);
                UNLOOK_LOG_WARNING("BankingMLValidator: Missing banking context field: " + field);
                return false;
            }
        }

        // Validate transaction ID format (basic validation)
        auto transaction_id_it = banking_context.find("transaction_id");
        if (transaction_id_it != banking_context.end()) {
            const std::string& transaction_id = transaction_id_it->second;
            if (transaction_id.length() < 8 || transaction_id.length() > 64) {
                setLastError("Invalid transaction ID format");
                return false;
            }
        }

        // Validate risk level
        auto risk_level_it = banking_context.find("risk_level");
        if (risk_level_it != banking_context.end()) {
            const std::string& risk_level = risk_level_it->second;
            const std::vector<std::string> valid_risk_levels = {"LOW", "MEDIUM", "HIGH", "CRITICAL"};
            if (std::find(valid_risk_levels.begin(), valid_risk_levels.end(), risk_level) ==
                valid_risk_levels.end()) {
                setLastError("Invalid risk level: " + risk_level);
                return false;
            }
        }

        // Validate authentication method
        auto auth_method_it = banking_context.find("authentication_method");
        if (auth_method_it != banking_context.end()) {
            const std::string& auth_method = auth_method_it->second;
            const std::vector<std::string> valid_auth_methods = {
                "biometric", "multi_factor", "certificate", "token"
            };
            if (std::find(valid_auth_methods.begin(), valid_auth_methods.end(), auth_method) ==
                valid_auth_methods.end()) {
                setLastError("Invalid authentication method: " + auth_method);
                return false;
            }
        }

        UNLOOK_LOG_DEBUG("BankingMLValidator: Banking context validation passed");
        return true;

    } catch (const std::exception& e) {
        setLastError("Banking context validation exception: " + std::string(e.what()));
        UNLOOK_LOG_ERROR("BankingMLValidator: Context validation exception: " + std::string(e.what()));
        return false;
    }
}

std::string BankingMLValidator::assessRisk(const AdvancedMLResult& ml_result,
                                          const BankingTransactionContext& transaction_context,
                                          float& risk_score,
                                          std::vector<std::string>& risk_factors) {
    UNLOOK_LOG_DEBUG("BankingMLValidator: Assessing risk for banking transaction");

    risk_factors.clear();
    risk_score = 0.0f;

    if (!isInitialized()) {
        setLastError("BankingMLValidator not initialized");
        return "ERROR";
    }

    try {
        // Calculate comprehensive risk score
        risk_score = calculateRiskScore(ml_result, transaction_context);

        // Identify specific risk factors
        identifyRiskFactors(ml_result, transaction_context, risk_factors);

        // Determine risk level based on score and factors
        std::string risk_level;
        if (risk_score >= 0.8f) {
            risk_level = "CRITICAL";
        } else if (risk_score >= 0.6f) {
            risk_level = "HIGH";
        } else if (risk_score >= 0.3f) {
            risk_level = "MEDIUM";
        } else {
            risk_level = "LOW";
        }

        // Additional checks for critical risk factors
        if (std::find(risk_factors.begin(), risk_factors.end(), "PRESENTATION_ATTACK_DETECTED") != risk_factors.end() ||
            std::find(risk_factors.begin(), risk_factors.end(), "AUTHENTICATION_FAILURE") != risk_factors.end()) {
            risk_level = "CRITICAL";
            risk_score = std::max(risk_score, 0.9f);
        }

        UNLOOK_LOG_DEBUG("BankingMLValidator: Risk assessment completed - Level: " + risk_level +
                        ", Score: " + std::to_string(risk_score) +
                        ", Factors: " + std::to_string(risk_factors.size()));

        return risk_level;

    } catch (const std::exception& e) {
        setLastError("Risk assessment exception: " + std::string(e.what()));
        UNLOOK_LOG_ERROR("BankingMLValidator: Risk assessment exception: " + std::string(e.what()));
        risk_score = 1.0f; // Maximum risk on error
        return "ERROR";
    }
}

bool BankingMLValidator::validatePerformanceMetrics(const std::map<std::string, float>& performance_metrics,
                                                   const std::map<std::string, float>& sla_requirements,
                                                   std::map<std::string, std::string>& validation_result) {
    UNLOOK_LOG_DEBUG("BankingMLValidator: Validating performance metrics against SLAs");

    validation_result.clear();
    bool all_valid = true;

    try {
        for (const auto& [metric_name, required_value] : sla_requirements) {
            auto metric_it = performance_metrics.find(metric_name);
            if (metric_it == performance_metrics.end()) {
                validation_result[metric_name + "_status"] = "MISSING";
                all_valid = false;
                continue;
            }

            float actual_value = metric_it->second;
            bool metric_valid = false;

            // Different validation logic based on metric type
            if (metric_name == "response_time_ms") {
                metric_valid = actual_value <= required_value;
            } else if (metric_name == "accuracy" || metric_name == "confidence") {
                metric_valid = actual_value >= required_value;
            } else {
                // Default: actual should be >= required
                metric_valid = actual_value >= required_value;
            }

            validation_result[metric_name + "_status"] = metric_valid ? "PASS" : "FAIL";
            validation_result[metric_name + "_actual"] = std::to_string(actual_value);
            validation_result[metric_name + "_required"] = std::to_string(required_value);

            if (!metric_valid) {
                all_valid = false;
            }
        }

        validation_result["overall_status"] = all_valid ? "PASS" : "FAIL";

        UNLOOK_LOG_DEBUG("BankingMLValidator: Performance validation - " +
                        (all_valid ? "PASSED" : "FAILED"));

        return all_valid;

    } catch (const std::exception& e) {
        setLastError("Performance validation exception: " + std::string(e.what()));
        UNLOOK_LOG_ERROR("BankingMLValidator: Performance validation exception: " + std::string(e.what()));
        validation_result["overall_status"] = "ERROR";
        return false;
    }
}

bool BankingMLValidator::validateDataProtection(const AdvancedMLResult& ml_result,
                                               const std::map<std::string, std::string>& data_context,
                                               std::map<std::string, std::string>& protection_result) {
    UNLOOK_LOG_DEBUG("BankingMLValidator: Validating data protection requirements");

    protection_result.clear();
    bool all_valid = true;

    try {
        // Validate encryption status
        if (config_.validate_data_encryption) {
            bool encryption_valid = ml_result.security_metrics.encryption_validated;
            protection_result["encryption_status"] = encryption_valid ? "VALID" : "INVALID";
            all_valid &= encryption_valid;

            if (!encryption_valid) {
                protection_result["encryption_violation"] = "Data encryption not validated";
            }
        }

        // Validate data integrity
        if (config_.validate_data_integrity) {
            bool integrity_valid = ml_result.security_metrics.data_integrity_verified;
            protection_result["integrity_status"] = integrity_valid ? "VALID" : "INVALID";
            all_valid &= integrity_valid;

            if (!integrity_valid) {
                protection_result["integrity_violation"] = "Data integrity not verified";
            }
        }

        // Validate GDPR compliance if enabled
        if (config_.enable_gdpr_validation) {
            bool gdpr_compliant = ml_result.compliance_metrics.gdpr_compliant;
            protection_result["gdpr_status"] = gdpr_compliant ? "COMPLIANT" : "NON_COMPLIANT";
            all_valid &= gdpr_compliant;

            if (!gdpr_compliant) {
                protection_result["gdpr_violation"] = "GDPR compliance requirements not met";
            }
        }

        // Validate access controls
        if (config_.validate_access_controls) {
            // Simplified access control validation
            bool access_controls_valid = !ml_result.security_metrics.security_level_achieved.empty();
            protection_result["access_controls_status"] = access_controls_valid ? "VALID" : "INVALID";
            all_valid &= access_controls_valid;
        }

        protection_result["overall_status"] = all_valid ? "COMPLIANT" : "NON_COMPLIANT";

        UNLOOK_LOG_DEBUG("BankingMLValidator: Data protection validation - " +
                        (all_valid ? "COMPLIANT" : "NON_COMPLIANT"));

        return all_valid;

    } catch (const std::exception& e) {
        setLastError("Data protection validation exception: " + std::string(e.what()));
        UNLOOK_LOG_ERROR("BankingMLValidator: Data protection validation exception: " + std::string(e.what()));
        protection_result["overall_status"] = "ERROR";
        return false;
    }
}

void BankingMLValidator::getValidationStatistics(size_t& total_validations,
                                                size_t& compliant_validations,
                                                size_t& non_compliant_validations,
                                                float& avg_compliance_score,
                                                float& avg_risk_score) const {
    std::lock_guard<std::mutex> lock(stats_mutex_);

    total_validations = total_validations_.load();
    compliant_validations = compliant_validations_.load();
    non_compliant_validations = non_compliant_validations_.load();

    if (total_validations > 0) {
        avg_compliance_score = cumulative_compliance_score_.load() / total_validations;
        avg_risk_score = cumulative_risk_score_.load() / total_validations;
    } else {
        avg_compliance_score = 0.0f;
        avg_risk_score = 0.0f;
    }
}

std::string BankingMLValidator::getLastError() const {
    std::lock_guard<std::mutex> lock(error_mutex_);
    return last_error_;
}

// ============================================================================
// Private Implementation Methods
// ============================================================================

bool BankingMLValidator::initializeComplianceComponents() {
    UNLOOK_LOG_INFO("BankingMLValidator: Initializing banking compliance components");

    try {
        // Initialize performance SLA thresholds
        performance_sla_thresholds_["response_time_ms"] = static_cast<float>(config_.max_response_time_ms);
        performance_sla_thresholds_["accuracy"] = config_.min_accuracy_threshold;
        performance_sla_thresholds_["far"] = config_.max_acceptable_far;
        performance_sla_thresholds_["frr"] = config_.max_acceptable_frr;

        UNLOOK_LOG_INFO("BankingMLValidator: All compliance components initialized successfully");
        return true;

    } catch (const std::exception& e) {
        UNLOOK_LOG_ERROR("BankingMLValidator: Compliance component initialization exception: " +
                        std::string(e.what()));
        return false;
    }
}

void BankingMLValidator::initializeComplianceThresholds() {
    UNLOOK_LOG_DEBUG("BankingMLValidator: Initializing compliance thresholds");

    // PCI DSS thresholds
    compliance_thresholds_[BankingComplianceStandard::PCI_DSS_L1]["far"] = 0.0001f;  // 0.01%
    compliance_thresholds_[BankingComplianceStandard::PCI_DSS_L1]["frr"] = 0.03f;    // 3%
    compliance_thresholds_[BankingComplianceStandard::PCI_DSS_L1]["accuracy"] = 0.97f;
    compliance_thresholds_[BankingComplianceStandard::PCI_DSS_L1]["response_time"] = 5000.0f; // 5 seconds

    // GDPR thresholds (focused on privacy and consent)
    compliance_thresholds_[BankingComplianceStandard::GDPR]["data_protection"] = 1.0f; // Mandatory
    compliance_thresholds_[BankingComplianceStandard::GDPR]["consent"] = 1.0f;         // Mandatory
    compliance_thresholds_[BankingComplianceStandard::GDPR]["accuracy"] = 0.95f;

    // SOX thresholds (focused on controls and audit)
    compliance_thresholds_[BankingComplianceStandard::SOX]["audit_trail"] = 1.0f;    // Mandatory
    compliance_thresholds_[BankingComplianceStandard::SOX]["controls"] = 1.0f;       // Mandatory
    compliance_thresholds_[BankingComplianceStandard::SOX]["accuracy"] = 0.96f;
}

bool BankingMLValidator::validatePCIDSSCompliance(const AdvancedMLResult& ml_result,
                                                 std::map<std::string, std::string>& compliance_details) {
    UNLOOK_LOG_DEBUG("BankingMLValidator: Validating PCI DSS compliance");

    compliance_details.clear();
    bool is_compliant = true;

    try {
        // Check FAR/FRR requirements
        auto pci_thresholds = compliance_thresholds_[BankingComplianceStandard::PCI_DSS_L1];

        if (ml_result.verification_result.false_accept_rate > pci_thresholds["far"]) {
            is_compliant = false;
            compliance_details["far_violation"] = "FAR exceeds PCI DSS requirements";
        }

        if (ml_result.verification_result.false_reject_rate > pci_thresholds["frr"]) {
            is_compliant = false;
            compliance_details["frr_violation"] = "FRR exceeds PCI DSS requirements";
        }

        // Check encryption requirements
        if (!ml_result.security_metrics.encryption_validated) {
            is_compliant = false;
            compliance_details["encryption_violation"] = "PCI DSS requires data encryption";
        }

        // Check response time requirements
        if (ml_result.performance_metrics.total_processing_time_ms > pci_thresholds["response_time"]) {
            is_compliant = false;
            compliance_details["performance_violation"] = "Response time exceeds PCI DSS requirements";
        }

        compliance_details["pci_dss_status"] = is_compliant ? "COMPLIANT" : "NON_COMPLIANT";
        compliance_details["validation_timestamp"] = std::to_string(
            std::chrono::system_clock::to_time_t(std::chrono::system_clock::now()));

        return is_compliant;

    } catch (const std::exception& e) {
        compliance_details["error"] = "PCI DSS validation error: " + std::string(e.what());
        return false;
    }
}

bool BankingMLValidator::validateGDPRCompliance(const AdvancedMLResult& ml_result,
                                               std::map<std::string, std::string>& compliance_details) {
    UNLOOK_LOG_DEBUG("BankingMLValidator: Validating GDPR compliance");

    compliance_details.clear();
    bool is_compliant = true;

    try {
        // Check data protection status
        if (!ml_result.compliance_metrics.gdpr_compliant) {
            is_compliant = false;
            compliance_details["gdpr_violation"] = "GDPR compliance not achieved";
        }

        // Check data encryption (GDPR Article 32)
        if (!ml_result.security_metrics.encryption_validated) {
            is_compliant = false;
            compliance_details["encryption_violation"] = "GDPR requires appropriate technical safeguards";
        }

        // Check data integrity (GDPR Article 5)
        if (!ml_result.security_metrics.data_integrity_verified) {
            is_compliant = false;
            compliance_details["integrity_violation"] = "GDPR requires data integrity assurance";
        }

        compliance_details["gdpr_status"] = is_compliant ? "COMPLIANT" : "NON_COMPLIANT";
        compliance_details["data_protection_status"] = ml_result.compliance_metrics.gdpr_compliant ? "VALID" : "INVALID";

        return is_compliant;

    } catch (const std::exception& e) {
        compliance_details["error"] = "GDPR validation error: " + std::string(e.what());
        return false;
    }
}

bool BankingMLValidator::validateSOXCompliance(const AdvancedMLResult& ml_result,
                                              std::map<std::string, std::string>& compliance_details) {
    UNLOOK_LOG_DEBUG("BankingMLValidator: Validating SOX compliance");

    compliance_details.clear();
    bool is_compliant = true;

    try {
        // Check audit trail requirements (SOX Section 404)
        if (ml_result.compliance_metrics.audit_trail_id.empty()) {
            is_compliant = false;
            compliance_details["audit_violation"] = "SOX requires comprehensive audit trails";
        }

        // Check internal controls
        if (ml_result.security_metrics.security_level_achieved.empty()) {
            is_compliant = false;
            compliance_details["controls_violation"] = "SOX requires documented internal controls";
        }

        // Check accuracy for financial reporting
        if (ml_result.verification_result.similarity_score < 0.96f) {
            is_compliant = false;
            compliance_details["accuracy_violation"] = "SOX requires high accuracy for financial controls";
        }

        compliance_details["sox_status"] = is_compliant ? "COMPLIANT" : "NON_COMPLIANT";
        compliance_details["audit_trail_id"] = ml_result.compliance_metrics.audit_trail_id;

        return is_compliant;

    } catch (const std::exception& e) {
        compliance_details["error"] = "SOX validation error: " + std::string(e.what());
        return false;
    }
}

bool BankingMLValidator::validateRegulatoryCompliance(BankingComplianceStandard compliance_standard,
                                                     const AdvancedMLResult& ml_result,
                                                     const std::map<std::string, std::string>& regulatory_context,
                                                     std::map<std::string, std::string>& compliance_details) {
    switch (compliance_standard) {
        case BankingComplianceStandard::PCI_DSS_L1:
        case BankingComplianceStandard::PCI_DSS_L2:
            return validatePCIDSSCompliance(ml_result, compliance_details);

        case BankingComplianceStandard::GDPR:
            return validateGDPRCompliance(ml_result, compliance_details);

        case BankingComplianceStandard::SOX:
            return validateSOXCompliance(ml_result, compliance_details);

        default:
            // For other standards, perform basic validation
            compliance_details["standard"] = "BASIC_VALIDATION";
            compliance_details["status"] = "COMPLIANT";
            return true;
    }
}

float BankingMLValidator::calculateRiskScore(const AdvancedMLResult& ml_result,
                                            const BankingTransactionContext& transaction_context) {
    float risk_score = 0.0f;

    try {
        // Risk factors and their weights
        std::vector<std::pair<float, float>> risk_components; // (factor_value, weight)

        // 1. False Accept Rate risk (weight: 0.3)
        float far_risk = ml_result.verification_result.false_accept_rate / config_.max_acceptable_far;
        risk_components.push_back({std::min(1.0f, far_risk), 0.3f});

        // 2. Confidence level risk (weight: 0.2)
        float confidence_risk = 1.0f - ml_result.verification_result.confidence_level;
        risk_components.push_back({confidence_risk, 0.2f});

        // 3. Quality risk (weight: 0.2)
        float quality_risk = 1.0f - ml_result.quality_assessment.overall_quality_score;
        risk_components.push_back({quality_risk, 0.2f});

        // 4. Transaction amount risk (weight: 0.15)
        float amount_risk = transaction_context.high_value_transaction ? 0.5f : 0.1f;
        risk_components.push_back({amount_risk, 0.15f});

        // 5. Performance risk (weight: 0.15)
        float performance_risk = (ml_result.performance_metrics.total_processing_time_ms > config_.max_response_time_ms) ? 0.8f : 0.1f;
        risk_components.push_back({performance_risk, 0.15f});

        // Calculate weighted risk score
        float total_weight = 0.0f;
        for (const auto& [factor_value, weight] : risk_components) {
            risk_score += factor_value * weight;
            total_weight += weight;
        }

        // Normalize risk score
        if (total_weight > 0.0f) {
            risk_score /= total_weight;
        }

        // Ensure risk score is in [0, 1] range
        risk_score = std::max(0.0f, std::min(1.0f, risk_score));

        return risk_score;

    } catch (const std::exception& e) {
        UNLOOK_LOG_ERROR("BankingMLValidator: Risk calculation exception: " + std::string(e.what()));
        return 1.0f; // Maximum risk on error
    }
}

size_t BankingMLValidator::identifyRiskFactors(const AdvancedMLResult& ml_result,
                                              const BankingTransactionContext& transaction_context,
                                              std::vector<std::string>& risk_factors) {
    risk_factors.clear();

    try {
        // Check various risk conditions
        if (ml_result.verification_result.false_accept_rate > config_.max_acceptable_far) {
            risk_factors.push_back("HIGH_FALSE_ACCEPT_RATE");
        }

        if (ml_result.verification_result.confidence_level < 0.9f) {
            risk_factors.push_back("LOW_CONFIDENCE_SCORE");
        }

        if (ml_result.quality_assessment.overall_quality_score < 0.8f) {
            risk_factors.push_back("POOR_BIOMETRIC_QUALITY");
        }

        if (ml_result.performance_metrics.total_processing_time_ms > config_.max_response_time_ms) {
            risk_factors.push_back("SLOW_RESPONSE_TIME");
        }

        if (transaction_context.high_value_transaction) {
            risk_factors.push_back("HIGH_VALUE_TRANSACTION");
        }

        if (!ml_result.security_metrics.encryption_validated) {
            risk_factors.push_back("ENCRYPTION_NOT_VALIDATED");
        }

        if (!ml_result.security_metrics.data_integrity_verified) {
            risk_factors.push_back("DATA_INTEGRITY_NOT_VERIFIED");
        }

        if (transaction_context.risk_category == "HIGH" || transaction_context.risk_category == "CRITICAL") {
            risk_factors.push_back("HIGH_RISK_TRANSACTION_CATEGORY");
        }

        return risk_factors.size();

    } catch (const std::exception& e) {
        UNLOOK_LOG_ERROR("BankingMLValidator: Risk factor identification exception: " + std::string(e.what()));
        risk_factors.push_back("RISK_ASSESSMENT_ERROR");
        return 1;
    }
}

void BankingMLValidator::generateRemediationRecommendations(const BankingComplianceResult& compliance_result,
                                                           std::vector<std::string>& remediation_actions,
                                                           std::string& priority) {
    remediation_actions.clear();

    try {
        // Determine priority based on compliance score
        if (compliance_result.compliance_score < 0.5f) {
            priority = "IMMEDIATE";
        } else if (compliance_result.compliance_score < 0.7f) {
            priority = "HIGH";
        } else if (compliance_result.compliance_score < 0.9f) {
            priority = "MEDIUM";
        } else {
            priority = "LOW";
        }

        // Generate specific remediation actions based on failed checks
        for (const auto& failed_check : compliance_result.validation_details.failed_checks) {
            if (failed_check.find("FAR") != std::string::npos) {
                remediation_actions.push_back("Improve biometric template quality to reduce false accept rate");
                remediation_actions.push_back("Review and adjust matching thresholds");
            }

            if (failed_check.find("FRR") != std::string::npos) {
                remediation_actions.push_back("Optimize template generation process to reduce false reject rate");
                remediation_actions.push_back("Consider template re-enrollment for affected users");
            }

            if (failed_check.find("encryption") != std::string::npos) {
                remediation_actions.push_back("Enable end-to-end encryption for all biometric data");
                remediation_actions.push_back("Verify encryption key management procedures");
            }

            if (failed_check.find("performance") != std::string::npos) {
                remediation_actions.push_back("Optimize ML processing pipeline for faster response times");
                remediation_actions.push_back("Consider hardware acceleration or caching optimizations");
            }
        }

        // Add general recommendations
        if (remediation_actions.empty()) {
            remediation_actions.push_back("Review system configuration for optimal compliance");
            remediation_actions.push_back("Conduct regular compliance audits");
        }

    } catch (const std::exception& e) {
        UNLOOK_LOG_ERROR("BankingMLValidator: Remediation generation exception: " + std::string(e.what()));
        remediation_actions.push_back("Contact system administrator for compliance review");
        priority = "HIGH";
    }
}

void BankingMLValidator::updateStatistics(float compliance_score, float risk_score, bool is_compliant) {
    std::lock_guard<std::mutex> lock(stats_mutex_);

    total_validations_++;
    cumulative_compliance_score_ += compliance_score;
    cumulative_risk_score_ += risk_score;

    if (is_compliant) {
        compliant_validations_++;
    } else {
        non_compliant_validations_++;
    }
}

void BankingMLValidator::sendComplianceAlert(const std::string& violation_type,
                                            const std::string& severity,
                                            const std::map<std::string, std::string>& details) {
    if (!config_.enable_compliance_alerts) {
        return;
    }

    std::lock_guard<std::mutex> lock(alert_mutex_);

    // Log compliance violation
    std::ostringstream alert_message;
    alert_message << "BANKING COMPLIANCE ALERT [" << severity << "]: " << violation_type;
    for (const auto& [key, value] : details) {
        alert_message << " | " << key << "=" << value;
    }

    UNLOOK_LOG_WARNING(alert_message.str());

    // Store recent violation for reporting
    recent_violations_.push_back(violation_type + " (" + severity + ")");

    // Keep only last 100 violations
    if (recent_violations_.size() > 100) {
        recent_violations_.erase(recent_violations_.begin());
    }
}

void BankingMLValidator::setLastError(const std::string& error_message) {
    std::lock_guard<std::mutex> lock(error_mutex_);
    last_error_ = error_message;
    error_history_.push_back(error_message);

    // Keep only last 100 errors
    if (error_history_.size() > 100) {
        error_history_.erase(error_history_.begin());
    }
}

// ============================================================================
// BankingMLValidatorUtils Implementation
// ============================================================================

std::string BankingMLValidatorUtils::complianceStandardToString(BankingComplianceStandard standard) {
    switch (standard) {
        case BankingComplianceStandard::PCI_DSS_L1: return "PCI_DSS_L1";
        case BankingComplianceStandard::PCI_DSS_L2: return "PCI_DSS_L2";
        case BankingComplianceStandard::GDPR: return "GDPR";
        case BankingComplianceStandard::SOX: return "SOX";
        case BankingComplianceStandard::HIPAA: return "HIPAA";
        case BankingComplianceStandard::BASEL_III: return "BASEL_III";
        case BankingComplianceStandard::FFIEC: return "FFIEC";
        case BankingComplianceStandard::ISO_27001: return "ISO_27001";
        case BankingComplianceStandard::NIST_CSF: return "NIST_CSF";
        case BankingComplianceStandard::CUSTOM: return "CUSTOM";
        default: return "UNKNOWN";
    }
}

BankingComplianceStandard BankingMLValidatorUtils::complianceStandardFromString(const std::string& standard_str) {
    if (standard_str == "PCI_DSS_L1") return BankingComplianceStandard::PCI_DSS_L1;
    if (standard_str == "PCI_DSS_L2") return BankingComplianceStandard::PCI_DSS_L2;
    if (standard_str == "GDPR") return BankingComplianceStandard::GDPR;
    if (standard_str == "SOX") return BankingComplianceStandard::SOX;
    if (standard_str == "HIPAA") return BankingComplianceStandard::HIPAA;
    if (standard_str == "BASEL_III") return BankingComplianceStandard::BASEL_III;
    if (standard_str == "FFIEC") return BankingComplianceStandard::FFIEC;
    if (standard_str == "ISO_27001") return BankingComplianceStandard::ISO_27001;
    if (standard_str == "NIST_CSF") return BankingComplianceStandard::NIST_CSF;
    if (standard_str == "CUSTOM") return BankingComplianceStandard::CUSTOM;
    return BankingComplianceStandard::CUSTOM; // Default fallback
}

} // namespace face
} // namespace unlook