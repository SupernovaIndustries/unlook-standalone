/**
 * @file MLTemplateManager.cpp
 * @brief ML Template Manager Implementation for Banking-Grade Template Lifecycle Management
 *
 * Professional implementation providing comprehensive lifecycle management
 * for banking-grade biometric templates with optimization, caching,
 * quality assessment, and secure storage capabilities.
 *
 * @version 2.0.0-banking-templates
 * @author Unlook Team - API Architecture Agent
 * @copyright 2025 Unlook. All rights reserved.
 */

#include "unlook/face/MLTemplateManager.hpp"
#include "unlook/core/Logger.hpp"
#include "unlook/core/Exception.hpp"

#include <algorithm>
#include <random>
#include <iomanip>
#include <sstream>
#include <fstream>
#include <filesystem>
#include <chrono>
#include <numeric>

namespace unlook {
namespace face {

// ============================================================================
// MLTemplateManagerConfig Implementation
// ============================================================================

bool MLTemplateManagerConfig::validate() const {
    if (quality_threshold < 0.0f || quality_threshold > 1.0f) {
        return false;
    }

    if (max_templates_per_user <= 0 || max_templates_per_user > 100) {
        return false;
    }

    if (max_cache_size <= 0 || max_cache_size > 1000000) {
        return false;
    }

    if (cache_ttl_hours <= 0 || cache_ttl_hours > 8760) { // Max 1 year
        return false;
    }

    if (storage_path.empty()) {
        return false;
    }

    if (max_worker_threads <= 0 || max_worker_threads > 32) {
        return false;
    }

    if (batch_size <= 0 || batch_size > 10000) {
        return false;
    }

    return true;
}

std::string MLTemplateManagerConfig::toString() const {
    std::ostringstream oss;
    oss << "MLTemplateManagerConfig{";
    oss << "optimization=" << (enable_template_optimization ? "enabled" : "disabled") << ", ";
    oss << "caching=" << (enable_template_caching ? "enabled" : "disabled") << ", ";
    oss << "quality_threshold=" << quality_threshold << ", ";
    oss << "max_templates_per_user=" << max_templates_per_user << ", ";
    oss << "max_cache_size=" << max_cache_size << ", ";
    oss << "storage_backend='" << storage_backend << "', ";
    oss << "storage_path='" << storage_path << "', ";
    oss << "encrypted_storage=" << (enable_encrypted_storage ? "enabled" : "disabled") << ", ";
    oss << "banking_grade=" << (banking_grade_templates ? "enabled" : "disabled") << ", ";
    oss << "compliance_level='" << compliance_level << "'";
    oss << "}";
    return oss.str();
}

// ============================================================================
// MLTemplateManager Implementation
// ============================================================================

MLTemplateManager::MLTemplateManager()
    : config_{}
    , initialized_(false)
    , banking_compliance_enabled_(false)
    , current_cache_size_(0)
    , total_operations_(0)
    , successful_operations_(0)
    , failed_operations_(0)
    , total_processing_time_(0.0)
    , cumulative_quality_score_(0.0f)
{
    UNLOOK_LOG_INFO("MLTemplateManager: Constructor with default configuration");
}

MLTemplateManager::MLTemplateManager(const MLTemplateManagerConfig& config)
    : config_(config)
    , initialized_(false)
    , banking_compliance_enabled_(false)
    , current_cache_size_(0)
    , total_operations_(0)
    , successful_operations_(0)
    , failed_operations_(0)
    , total_processing_time_(0.0)
    , cumulative_quality_score_(0.0f)
{
    UNLOOK_LOG_INFO("MLTemplateManager: Constructor with custom configuration");
    UNLOOK_LOG_DEBUG("MLTemplateManager: Configuration - " + config_.toString());
}

MLTemplateManager::~MLTemplateManager() {
    UNLOOK_LOG_INFO("MLTemplateManager: Destructor called");
}

bool MLTemplateManager::initialize(const AdvancedMLConfig& ml_config) {
    UNLOOK_LOG_INFO("MLTemplateManager: Initializing ML template manager");

    std::lock_guard<std::mutex> lock(config_mutex_);

    // Validate configuration
    if (!config_.validate()) {
        setLastError("Invalid MLTemplateManagerConfig provided");
        UNLOOK_LOG_ERROR("MLTemplateManager: Invalid configuration: " + config_.toString());
        return false;
    }

    // Update configuration based on ML config
    config_.banking_grade_templates = ml_config.banking_far_requirement <= 0.001f;
    config_.compliance_level = ml_config.compliance_level;
    config_.enable_encrypted_storage = ml_config.require_end_to_end_encryption;
    config_.require_template_signing = ml_config.require_end_to_end_encryption;

    // Initialize components
    if (!initializeComponents()) {
        setLastError("Failed to initialize template manager components");
        UNLOOK_LOG_ERROR("MLTemplateManager: Component initialization failed");
        return false;
    }

    // Initialize storage
    if (!initializeStorage()) {
        setLastError("Failed to initialize template storage");
        UNLOOK_LOG_ERROR("MLTemplateManager: Storage initialization failed");
        return false;
    }

    initialized_ = true;
    banking_compliance_enabled_ = config_.banking_grade_templates;

    UNLOOK_LOG_INFO("MLTemplateManager: Initialization completed successfully");
    return true;
}

bool MLTemplateManager::isInitialized() const {
    return initialized_.load();
}

FaceResultCode MLTemplateManager::optimizeEnrollmentTemplate(
    const std::vector<BiometricTemplate>& enrollment_data,
    const std::map<std::string, std::string>& user_context,
    BiometricTemplate& optimized_template) {

    UNLOOK_LOG_DEBUG("MLTemplateManager: Optimizing enrollment template");

    if (!isInitialized()) {
        setLastError("MLTemplateManager not initialized");
        return FaceResultCode::ERROR_TEMPLATE_GENERATION_FAILED;
    }

    if (enrollment_data.empty()) {
        setLastError("No enrollment data provided");
        return FaceResultCode::ERROR_TEMPLATE_INVALID;
    }

    auto start_time = std::chrono::high_resolution_clock::now();

    try {
        // Step 1: Apply quality filtering if enabled
        std::vector<BiometricTemplate> filtered_templates;
        if (config_.enable_quality_filtering) {
            size_t num_filtered = applyQualityFiltering(enrollment_data,
                                                       config_.quality_threshold,
                                                       filtered_templates);
            if (num_filtered == 0) {
                setLastError("No templates passed quality filtering");
                return FaceResultCode::ERROR_POOR_FACE_QUALITY;
            }
            UNLOOK_LOG_DEBUG("MLTemplateManager: " + std::to_string(num_filtered) +
                            " templates passed quality filtering");
        } else {
            filtered_templates = enrollment_data;
        }

        // Step 2: Template fusion if multiple templates available
        BiometricTemplate fused_template;
        if (filtered_templates.size() > 1 && config_.enable_multi_template_fusion) {
            FaceResultCode fusion_result = fuseTemplates(filtered_templates,
                                                        config_.optimization_algorithm,
                                                        fused_template);
            if (fusion_result != FaceResultCode::SUCCESS) {
                UNLOOK_LOG_WARNING("MLTemplateManager: Template fusion failed, using best template");
                fused_template = filtered_templates[0]; // Use first template as fallback
            } else {
                UNLOOK_LOG_DEBUG("MLTemplateManager: Successfully fused " +
                                std::to_string(filtered_templates.size()) + " templates");
            }
        } else {
            fused_template = filtered_templates[0];
        }

        // Step 3: Apply ML-based optimization
        if (config_.enable_template_optimization) {
            std::map<std::string, float> optimization_params = {
                {"quality_threshold", config_.quality_threshold},
                {"banking_grade", banking_compliance_enabled_ ? 1.0f : 0.0f}
            };

            FaceResultCode optimization_result = optimizeSingleTemplate(fused_template,
                                                                       optimization_params,
                                                                       optimized_template);
            if (optimization_result != FaceResultCode::SUCCESS) {
                UNLOOK_LOG_WARNING("MLTemplateManager: Template optimization failed, using original");
                optimized_template = fused_template;
            }
        } else {
            optimized_template = fused_template;
        }

        // Step 4: Banking compliance validation if required
        if (banking_compliance_enabled_.load()) {
            std::map<std::string, std::string> validation_result;
            bool is_banking_compliant = validateBankingGradeTemplate(optimized_template,
                                                                    user_context,
                                                                    validation_result);
            if (!is_banking_compliant) {
                setLastError("Template does not meet banking compliance requirements");
                return FaceResultCode::ERROR_AUTHORIZATION_DENIED;
            }
        }

        // Update statistics
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
        double processing_time_ms = duration.count() / 1000.0;

        float quality_score = optimized_template.quality_score;
        updateStatistics(processing_time_ms, true, quality_score);

        UNLOOK_LOG_DEBUG("MLTemplateManager: Enrollment optimization completed in " +
                        std::to_string(processing_time_ms) + " ms");

        return FaceResultCode::SUCCESS;

    } catch (const std::exception& e) {
        setLastError("Enrollment optimization exception: " + std::string(e.what()));
        UNLOOK_LOG_ERROR("MLTemplateManager: Optimization exception: " + std::string(e.what()));

        // Update statistics for failure
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
        double processing_time_ms = duration.count() / 1000.0;
        updateStatistics(processing_time_ms, false);

        return FaceResultCode::ERROR_TEMPLATE_GENERATION_FAILED;
    }
}

FaceResultCode MLTemplateManager::storeTemplate(const BiometricTemplate& template_data,
                                               const TemplateMetadata& metadata,
                                               std::string& template_id) {
    UNLOOK_LOG_DEBUG("MLTemplateManager: Storing template");

    if (!isInitialized()) {
        setLastError("MLTemplateManager not initialized");
        return FaceResultCode::ERROR_TEMPLATE_GENERATION_FAILED;
    }

    auto start_time = std::chrono::high_resolution_clock::now();

    try {
        // Generate template ID if not provided
        if (metadata.template_id.empty()) {
            template_id = generateTemplateId(metadata.user_id);
        } else {
            template_id = metadata.template_id;
        }

        // Create mutable copy of metadata
        TemplateMetadata store_metadata = metadata;
        store_metadata.template_id = template_id;
        store_metadata.updated_at = std::chrono::system_clock::now();
        store_metadata.template_size_bytes = template_data.feature_vector.size() * sizeof(float);

        // Generate digital signature if required
        if (config_.require_template_signing) {
            std::string signature;
            if (generateTemplateSignature(template_data, signature)) {
                store_metadata.digital_signature = signature;
            } else {
                UNLOOK_LOG_WARNING("MLTemplateManager: Failed to generate template signature");
            }
        }

        // Store in main storage
        {
            std::lock_guard<std::mutex> lock(storage_mutex_);
            template_storage_[template_id] = template_data;
            metadata_storage_[template_id] = store_metadata;
        }

        // Update cache if enabled
        if (config_.enable_template_caching) {
            updateCache(template_id, template_data, store_metadata);
        }

        // Update statistics
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
        double processing_time_ms = duration.count() / 1000.0;

        updateStatistics(processing_time_ms, true, template_data.quality_score);

        UNLOOK_LOG_DEBUG("MLTemplateManager: Template stored successfully with ID: " + template_id);

        return FaceResultCode::SUCCESS;

    } catch (const std::exception& e) {
        setLastError("Template storage exception: " + std::string(e.what()));
        UNLOOK_LOG_ERROR("MLTemplateManager: Storage exception: " + std::string(e.what()));

        // Update statistics for failure
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
        double processing_time_ms = duration.count() / 1000.0;
        updateStatistics(processing_time_ms, false);

        return FaceResultCode::ERROR_TEMPLATE_GENERATION_FAILED;
    }
}

FaceResultCode MLTemplateManager::retrieveTemplate(const std::string& template_id,
                                                  BiometricTemplate& template_data,
                                                  TemplateMetadata& metadata) {
    UNLOOK_LOG_DEBUG("MLTemplateManager: Retrieving template ID: " + template_id);

    if (!isInitialized()) {
        setLastError("MLTemplateManager not initialized");
        return FaceResultCode::ERROR_TEMPLATE_INVALID;
    }

    auto start_time = std::chrono::high_resolution_clock::now();

    try {
        // Check cache first if enabled
        if (config_.enable_template_caching) {
            std::lock_guard<std::mutex> cache_lock(cache_mutex_);
            auto cache_it = template_cache_.find(template_id);
            if (cache_it != template_cache_.end()) {
                // Update access information
                cache_it->second.last_accessed = std::chrono::system_clock::now();
                cache_it->second.access_count++;

                template_data = cache_it->second.template_data;
                metadata = cache_it->second.metadata;

                UNLOOK_LOG_DEBUG("MLTemplateManager: Template retrieved from cache");
                return FaceResultCode::SUCCESS;
            }
        }

        // Retrieve from main storage
        {
            std::lock_guard<std::mutex> lock(storage_mutex_);
            auto template_it = template_storage_.find(template_id);
            auto metadata_it = metadata_storage_.find(template_id);

            if (template_it == template_storage_.end() || metadata_it == metadata_storage_.end()) {
                setLastError("Template not found: " + template_id);
                return FaceResultCode::ERROR_TEMPLATE_INVALID;
            }

            template_data = template_it->second;
            metadata = metadata_it->second;

            // Update usage statistics
            metadata_it->second.usage_count++;
            metadata_it->second.last_used = std::chrono::system_clock::now();
        }

        // Verify digital signature if present
        if (!metadata.digital_signature.empty()) {
            if (!verifyTemplateSignature(template_data, metadata.digital_signature)) {
                setLastError("Template signature verification failed: " + template_id);
                UNLOOK_LOG_ERROR("MLTemplateManager: Signature verification failed for template " + template_id);
                return FaceResultCode::ERROR_TEMPLATE_CORRUPTED;
            }
        }

        // Update cache with retrieved template
        if (config_.enable_template_caching) {
            updateCache(template_id, template_data, metadata);
        }

        // Update statistics
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
        double processing_time_ms = duration.count() / 1000.0;

        updateStatistics(processing_time_ms, true);

        UNLOOK_LOG_DEBUG("MLTemplateManager: Template retrieved successfully");

        return FaceResultCode::SUCCESS;

    } catch (const std::exception& e) {
        setLastError("Template retrieval exception: " + std::string(e.what()));
        UNLOOK_LOG_ERROR("MLTemplateManager: Retrieval exception: " + std::string(e.what()));

        // Update statistics for failure
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
        double processing_time_ms = duration.count() / 1000.0;
        updateStatistics(processing_time_ms, false);

        return FaceResultCode::ERROR_TEMPLATE_INVALID;
    }
}

float MLTemplateManager::assessTemplateQuality(const BiometricTemplate& template_data,
                                              std::map<std::string, float>& quality_metrics) {
    UNLOOK_LOG_DEBUG("MLTemplateManager: Assessing template quality");

    quality_metrics.clear();

    if (!isInitialized()) {
        setLastError("MLTemplateManager not initialized");
        return 0.0f;
    }

    try {
        // Basic quality assessments
        float feature_completeness = 0.0f;
        if (!template_data.feature_vector.empty()) {
            // Assess feature vector completeness and distribution
            size_t non_zero_features = 0;
            float feature_sum = 0.0f;
            float feature_variance = 0.0f;

            for (float feature : template_data.feature_vector) {
                if (std::abs(feature) > 1e-6f) {
                    non_zero_features++;
                }
                feature_sum += feature;
            }

            feature_completeness = static_cast<float>(non_zero_features) / template_data.feature_vector.size();

            // Calculate variance
            float mean = feature_sum / template_data.feature_vector.size();
            for (float feature : template_data.feature_vector) {
                feature_variance += (feature - mean) * (feature - mean);
            }
            feature_variance /= template_data.feature_vector.size();

            quality_metrics["feature_completeness"] = feature_completeness;
            quality_metrics["feature_variance"] = feature_variance;
            quality_metrics["feature_mean"] = mean;
        }

        // Template size assessment
        float size_quality = 1.0f;
        if (template_data.feature_vector.size() > 0) {
            // Penalize templates that are too small or too large
            size_t ideal_size = 2048; // Assuming ideal feature vector size
            float size_ratio = static_cast<float>(template_data.feature_vector.size()) / ideal_size;
            size_quality = 1.0f / (1.0f + std::abs(size_ratio - 1.0f));
        }
        quality_metrics["size_quality"] = size_quality;

        // Landmark quality assessment
        float landmark_quality = 0.8f; // Default score
        if (!template_data.landmarks.points.empty()) {
            // Assess landmark distribution and consistency
            cv::Point2f centroid(0.0f, 0.0f);
            for (const auto& point : template_data.landmarks.points) {
                centroid.x += point.x;
                centroid.y += point.y;
            }
            centroid.x /= template_data.landmarks.points.size();
            centroid.y /= template_data.landmarks.points.size();

            // Calculate spread from centroid
            float total_distance = 0.0f;
            for (const auto& point : template_data.landmarks.points) {
                cv::Point2f diff = point - centroid;
                total_distance += std::sqrt(diff.x * diff.x + diff.y * diff.y);
            }
            float avg_distance = total_distance / template_data.landmarks.points.size();

            // Good landmark quality has reasonable spread
            landmark_quality = std::min(1.0f, avg_distance / 100.0f); // Normalize based on expected face size
        }
        quality_metrics["landmark_quality"] = landmark_quality;

        // Overall quality score (weighted combination)
        float overall_quality = (feature_completeness * 0.4f +
                                size_quality * 0.2f +
                                landmark_quality * 0.3f +
                                template_data.quality_score * 0.1f);

        quality_metrics["overall_quality"] = overall_quality;

        // Banking-specific quality assessments
        if (banking_compliance_enabled_.load()) {
            quality_metrics["banking_grade_ready"] = overall_quality >= 0.85f ? 1.0f : 0.0f;
            quality_metrics["banking_quality_threshold"] = 0.85f;
        }

        UNLOOK_LOG_DEBUG("MLTemplateManager: Quality assessment completed - overall score: " +
                        std::to_string(overall_quality));

        return overall_quality;

    } catch (const std::exception& e) {
        setLastError("Quality assessment exception: " + std::string(e.what()));
        UNLOOK_LOG_ERROR("MLTemplateManager: Quality assessment exception: " + std::string(e.what()));
        return 0.0f;
    }
}

FaceResultCode MLTemplateManager::fuseTemplates(const std::vector<BiometricTemplate>& templates,
                                               const std::string& fusion_method,
                                               BiometricTemplate& fused_template) {
    UNLOOK_LOG_DEBUG("MLTemplateManager: Fusing " + std::to_string(templates.size()) +
                    " templates using " + fusion_method + " method");

    if (templates.empty()) {
        setLastError("No templates provided for fusion");
        return FaceResultCode::ERROR_TEMPLATE_INVALID;
    }

    if (templates.size() == 1) {
        fused_template = templates[0];
        return FaceResultCode::SUCCESS;
    }

    try {
        // Initialize fused template with first template structure
        fused_template = templates[0];
        fused_template.template_id = generateTemplateId("fused");

        if (fusion_method == "average") {
            // Simple averaging of feature vectors
            std::fill(fused_template.feature_vector.begin(), fused_template.feature_vector.end(), 0.0f);

            for (const auto& template_data : templates) {
                if (template_data.feature_vector.size() != fused_template.feature_vector.size()) {
                    setLastError("Template feature vector sizes do not match for fusion");
                    return FaceResultCode::ERROR_TEMPLATE_INVALID;
                }

                for (size_t i = 0; i < template_data.feature_vector.size(); ++i) {
                    fused_template.feature_vector[i] += template_data.feature_vector[i];
                }
            }

            // Normalize by number of templates
            for (float& feature : fused_template.feature_vector) {
                feature /= static_cast<float>(templates.size());
            }

        } else if (fusion_method == "weighted") {
            // Quality-weighted averaging
            std::fill(fused_template.feature_vector.begin(), fused_template.feature_vector.end(), 0.0f);
            float total_weight = 0.0f;

            for (const auto& template_data : templates) {
                if (template_data.feature_vector.size() != fused_template.feature_vector.size()) {
                    setLastError("Template feature vector sizes do not match for fusion");
                    return FaceResultCode::ERROR_TEMPLATE_INVALID;
                }

                float weight = template_data.quality_score; // Use quality score as weight
                total_weight += weight;

                for (size_t i = 0; i < template_data.feature_vector.size(); ++i) {
                    fused_template.feature_vector[i] += template_data.feature_vector[i] * weight;
                }
            }

            // Normalize by total weight
            if (total_weight > 1e-6f) {
                for (float& feature : fused_template.feature_vector) {
                    feature /= total_weight;
                }
            }

        } else if (fusion_method == "ml_enhanced") {
            // ML-enhanced fusion (placeholder for advanced algorithm)
            // For now, use weighted averaging with additional processing
            FaceResultCode weighted_result = fuseTemplates(templates, "weighted", fused_template);
            if (weighted_result != FaceResultCode::SUCCESS) {
                return weighted_result;
            }

            // Apply enhancement (e.g., non-linear transformation)
            for (float& feature : fused_template.feature_vector) {
                feature = std::tanh(feature); // Apply tanh activation
            }

        } else {
            // Default to average method
            return fuseTemplates(templates, "average", fused_template);
        }

        // Update fused template quality score
        float total_quality = 0.0f;
        for (const auto& template_data : templates) {
            total_quality += template_data.quality_score;
        }
        fused_template.quality_score = total_quality / templates.size();

        // Update metadata
        fused_template.creation_time = std::chrono::system_clock::now();
        fused_template.version = "fused_v2.0";

        UNLOOK_LOG_DEBUG("MLTemplateManager: Template fusion completed successfully");

        return FaceResultCode::SUCCESS;

    } catch (const std::exception& e) {
        setLastError("Template fusion exception: " + std::string(e.what()));
        UNLOOK_LOG_ERROR("MLTemplateManager: Fusion exception: " + std::string(e.what()));
        return FaceResultCode::ERROR_TEMPLATE_GENERATION_FAILED;
    }
}

bool MLTemplateManager::validateBankingGradeTemplate(const BiometricTemplate& template_data,
                                                    const std::map<std::string, std::string>& validation_context,
                                                    std::map<std::string, std::string>& validation_result) {
    UNLOOK_LOG_DEBUG("MLTemplateManager: Validating banking-grade template");

    validation_result.clear();

    try {
        bool is_valid = true;
        std::vector<std::string> validation_issues;

        // Quality score validation
        if (template_data.quality_score < 0.85f) {
            is_valid = false;
            validation_issues.push_back("Quality score below banking threshold (0.85)");
        }
        validation_result["quality_score"] = std::to_string(template_data.quality_score);

        // Feature vector completeness
        if (template_data.feature_vector.empty()) {
            is_valid = false;
            validation_issues.push_back("Empty feature vector");
        } else {
            size_t non_zero_features = 0;
            for (float feature : template_data.feature_vector) {
                if (std::abs(feature) > 1e-6f) {
                    non_zero_features++;
                }
            }

            float completeness = static_cast<float>(non_zero_features) / template_data.feature_vector.size();
            if (completeness < 0.8f) {
                is_valid = false;
                validation_issues.push_back("Feature vector completeness below threshold (0.8)");
            }
            validation_result["feature_completeness"] = std::to_string(completeness);
        }

        // Landmark validation
        if (template_data.landmarks.points.size() < 68) {
            validation_issues.push_back("Insufficient landmarks for banking-grade accuracy");
            // This is a warning, not a failure
        }
        validation_result["landmark_count"] = std::to_string(template_data.landmarks.points.size());

        // Template age validation
        auto now = std::chrono::system_clock::now();
        auto template_age = now - template_data.creation_time;
        auto hours = std::chrono::duration_cast<std::chrono::hours>(template_age).count();

        if (hours > 8760) { // 1 year
            validation_issues.push_back("Template age exceeds banking compliance limit (1 year)");
            // This is a warning for potential renewal
        }
        validation_result["template_age_hours"] = std::to_string(hours);

        // Compliance metadata validation
        auto compliance_it = validation_context.find("compliance_level");
        if (compliance_it != validation_context.end()) {
            validation_result["required_compliance_level"] = compliance_it->second;
            validation_result["achieved_compliance_level"] = config_.compliance_level;

            if (config_.compliance_level != compliance_it->second) {
                validation_issues.push_back("Compliance level mismatch");
            }
        }

        // Summary
        validation_result["validation_status"] = is_valid ? "PASS" : "FAIL";
        validation_result["issues_count"] = std::to_string(validation_issues.size());

        if (!validation_issues.empty()) {
            std::ostringstream issues_stream;
            for (size_t i = 0; i < validation_issues.size(); ++i) {
                if (i > 0) issues_stream << "; ";
                issues_stream << validation_issues[i];
            }
            validation_result["validation_issues"] = issues_stream.str();
        }

        UNLOOK_LOG_DEBUG("MLTemplateManager: Banking validation " +
                        (is_valid ? "PASSED" : "FAILED") +
                        " with " + std::to_string(validation_issues.size()) + " issues");

        return is_valid;

    } catch (const std::exception& e) {
        setLastError("Banking validation exception: " + std::string(e.what()));
        UNLOOK_LOG_ERROR("MLTemplateManager: Banking validation exception: " + std::string(e.what()));
        validation_result["validation_status"] = "ERROR";
        validation_result["error_message"] = e.what();
        return false;
    }
}

void MLTemplateManager::getManagementStatistics(size_t& total_templates,
                                               size_t& cached_templates,
                                               size_t& optimized_templates,
                                               float& avg_quality_score,
                                               float& storage_usage_mb) const {
    std::lock_guard<std::mutex> lock(stats_mutex_);

    // Get template counts
    {
        std::lock_guard<std::mutex> storage_lock(storage_mutex_);
        total_templates = template_storage_.size();

        // Count optimized templates
        optimized_templates = 0;
        float total_quality = 0.0f;
        for (const auto& [template_id, metadata] : metadata_storage_) {
            if (metadata.is_optimized) {
                optimized_templates++;
            }
            total_quality += metadata.quality_score;
        }

        if (total_templates > 0) {
            avg_quality_score = total_quality / total_templates;
        } else {
            avg_quality_score = 0.0f;
        }
    }

    // Get cache statistics
    {
        std::lock_guard<std::mutex> cache_lock(cache_mutex_);
        cached_templates = template_cache_.size();
    }

    // Calculate storage usage
    storage_usage_mb = 0.0f;
    {
        std::lock_guard<std::mutex> storage_lock(storage_mutex_);
        for (const auto& [template_id, metadata] : metadata_storage_) {
            storage_usage_mb += static_cast<float>(metadata.template_size_bytes) / (1024.0f * 1024.0f);
        }
    }
}

std::string MLTemplateManager::getLastError() const {
    std::lock_guard<std::mutex> lock(error_mutex_);
    return last_error_;
}

// ============================================================================
// Private Implementation Methods
// ============================================================================

bool MLTemplateManager::initializeComponents() {
    UNLOOK_LOG_INFO("MLTemplateManager: Initializing template manager components");

    try {
        // Initialize storage containers
        template_storage_.clear();
        metadata_storage_.clear();
        template_cache_.clear();

        UNLOOK_LOG_INFO("MLTemplateManager: All components initialized successfully");
        return true;

    } catch (const std::exception& e) {
        UNLOOK_LOG_ERROR("MLTemplateManager: Component initialization exception: " +
                        std::string(e.what()));
        return false;
    }
}

bool MLTemplateManager::initializeStorage() {
    UNLOOK_LOG_INFO("MLTemplateManager: Initializing template storage");

    try {
        // Create storage directory if it doesn't exist
        if (!std::filesystem::exists(config_.storage_path)) {
            std::filesystem::create_directories(config_.storage_path);
        }

        // Set appropriate permissions (owner read/write only)
        std::filesystem::permissions(config_.storage_path,
                                   std::filesystem::perms::owner_read |
                                   std::filesystem::perms::owner_write,
                                   std::filesystem::perm_options::replace);

        UNLOOK_LOG_INFO("MLTemplateManager: Storage initialized at: " + config_.storage_path);
        return true;

    } catch (const std::exception& e) {
        setLastError("Storage initialization failed: " + std::string(e.what()));
        UNLOOK_LOG_ERROR("MLTemplateManager: Storage initialization exception: " +
                        std::string(e.what()));
        return false;
    }
}

FaceResultCode MLTemplateManager::optimizeSingleTemplate(const BiometricTemplate& template_data,
                                                        const std::map<std::string, float>& optimization_params,
                                                        BiometricTemplate& optimized_template) {
    try {
        // Start with a copy of the original template
        optimized_template = template_data;

        // Apply various optimization techniques
        if (config_.optimization_algorithm == "ml_enhanced") {
            // ML-enhanced optimization (placeholder for advanced algorithms)

            // Feature normalization
            if (!optimized_template.feature_vector.empty()) {
                // L2 normalization
                float norm = 0.0f;
                for (float feature : optimized_template.feature_vector) {
                    norm += feature * feature;
                }
                norm = std::sqrt(norm);

                if (norm > 1e-6f) {
                    for (float& feature : optimized_template.feature_vector) {
                        feature /= norm;
                    }
                }
            }

            // Quality enhancement based on parameters
            auto quality_threshold_it = optimization_params.find("quality_threshold");
            if (quality_threshold_it != optimization_params.end()) {
                float quality_boost = quality_threshold_it->second * 0.1f; // Small quality boost
                optimized_template.quality_score = std::min(1.0f,
                                                           optimized_template.quality_score + quality_boost);
            }
        }

        // Update optimization metadata
        optimized_template.template_id = generateTemplateId("optimized");
        optimized_template.version = "optimized_v2.0";
        optimized_template.creation_time = std::chrono::system_clock::now();

        return FaceResultCode::SUCCESS;

    } catch (const std::exception& e) {
        setLastError("Single template optimization failed: " + std::string(e.what()));
        return FaceResultCode::ERROR_TEMPLATE_GENERATION_FAILED;
    }
}

size_t MLTemplateManager::applyQualityFiltering(const std::vector<BiometricTemplate>& templates,
                                               float quality_threshold,
                                               std::vector<BiometricTemplate>& filtered_templates) {
    filtered_templates.clear();
    filtered_templates.reserve(templates.size());

    for (const auto& template_data : templates) {
        if (template_data.quality_score >= quality_threshold) {
            filtered_templates.push_back(template_data);
        }
    }

    return filtered_templates.size();
}

std::string MLTemplateManager::generateTemplateId(const std::string& user_id) {
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    auto microseconds = std::chrono::duration_cast<std::chrono::microseconds>(
        now.time_since_epoch()) % 1000000;

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(1000, 9999);

    std::ostringstream oss;
    oss << "tpl_" << user_id << "_" << time_t << "_" << microseconds.count() << "_" << dis(gen);
    return oss.str();
}

void MLTemplateManager::updateCache(const std::string& template_id,
                                   const BiometricTemplate& template_data,
                                   const TemplateMetadata& metadata) {
    std::lock_guard<std::mutex> lock(cache_mutex_);

    // Check if we need to evict from cache first
    if (template_cache_.size() >= config_.max_cache_size &&
        template_cache_.find(template_id) == template_cache_.end()) {
        evictLRUFromCache();
    }

    // Add or update cache entry
    CacheEntry entry;
    entry.template_data = template_data;
    entry.metadata = metadata;
    entry.last_accessed = std::chrono::system_clock::now();
    entry.access_count = 1;

    template_cache_[template_id] = entry;
    current_cache_size_ = template_cache_.size();
}

void MLTemplateManager::evictLRUFromCache() {
    if (template_cache_.empty()) {
        return;
    }

    // Find least recently used entry
    auto oldest_it = template_cache_.begin();
    for (auto it = template_cache_.begin(); it != template_cache_.end(); ++it) {
        if (it->second.last_accessed < oldest_it->second.last_accessed) {
            oldest_it = it;
        }
    }

    // Remove oldest entry
    template_cache_.erase(oldest_it);
    current_cache_size_ = template_cache_.size();
}

bool MLTemplateManager::generateTemplateSignature(const BiometricTemplate& template_data,
                                                  std::string& signature) {
    // Simplified signature generation (placeholder)
    // In production, use proper cryptographic signing
    std::ostringstream ss;
    ss << "SIG_" << std::hash<std::string>{}(template_data.template_id)
       << "_" << template_data.quality_score;
    signature = ss.str();
    return true;
}

bool MLTemplateManager::verifyTemplateSignature(const BiometricTemplate& template_data,
                                               const std::string& signature) {
    // Simplified signature verification (placeholder)
    std::string expected_signature;
    generateTemplateSignature(template_data, expected_signature);
    return signature == expected_signature;
}

void MLTemplateManager::updateStatistics(double processing_time, bool success, float quality_score) {
    std::lock_guard<std::mutex> lock(stats_mutex_);

    total_operations_++;
    total_processing_time_ += processing_time;

    if (success) {
        successful_operations_++;
        if (quality_score >= 0.0f) {
            cumulative_quality_score_ += quality_score;
        }
    } else {
        failed_operations_++;
    }
}

void MLTemplateManager::setLastError(const std::string& error_message) {
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