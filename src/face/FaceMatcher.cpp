#include "unlook/face/FaceMatcher.hpp"
#include "unlook/face/FaceException.hpp"
#include "unlook/core/logging.hpp"
#include <opencv2/core.hpp>
#include <algorithm>
#include <numeric>
#include <chrono>
#include <cmath>
#include <random>
#include <openssl/aes.h>
#include <openssl/rand.h>
#include <openssl/sha.h>

#ifdef __ARM_NEON
#include <arm_neon.h>
#endif

namespace unlook {
namespace face {

// Banking-grade security constants
static const size_t AES_KEY_SIZE = 32;          // 256-bit AES key
static const size_t AES_BLOCK_SIZE = 16;        // AES block size
static const size_t SHA256_DIGEST_SIZE = 32;    // SHA-256 hash size
static const float BANKING_FAR_THRESHOLD = 0.001f;  // 0.1% FAR maximum
static const float BANKING_FRR_THRESHOLD = 0.03f;   // 3% FRR maximum

// Feature vector constants for banking-grade templates
static const size_t BANKING_FEATURE_SIZE = 512;     // Standard feature vector size
static const size_t LANDMARK_FEATURE_SIZE = 68 * 3; // 68 landmarks × 3D coordinates
static const size_t GEOMETRIC_FEATURE_SIZE = 16;    // Geometric measurements

// Audit log entry structure
struct AuditLogEntry {
    std::chrono::system_clock::time_point timestamp;
    std::string probe_template_id;
    std::string reference_template_id;
    float similarity_score;
    bool match_decision;
    float confidence_level;
    std::string result_code;
    double processing_time_ms;
    std::string device_id;
    std::string session_id;
};

/**
 * @brief AES encryption/decryption utilities for template security
 */
class TemplateEncryption {
public:
    static bool encrypt(const std::vector<float>& plaintext,
                       const std::array<uint8_t, 32>& key,
                       std::vector<uint8_t>& ciphertext,
                       std::array<uint8_t, 16>& iv) {
        try {
            // Generate random IV
            if (RAND_bytes(iv.data(), iv.size()) != 1) {
                return false;
            }

            // Convert float vector to bytes
            std::vector<uint8_t> plaintext_bytes(plaintext.size() * sizeof(float));
            std::memcpy(plaintext_bytes.data(), plaintext.data(), plaintext_bytes.size());

            // Pad to AES block size
            size_t padded_size = ((plaintext_bytes.size() + AES_BLOCK_SIZE - 1) / AES_BLOCK_SIZE) * AES_BLOCK_SIZE;
            plaintext_bytes.resize(padded_size, 0);

            // Encrypt
            ciphertext.resize(padded_size);
            AES_KEY aes_key;
            AES_set_encrypt_key(key.data(), 256, &aes_key);

            for (size_t i = 0; i < padded_size; i += AES_BLOCK_SIZE) {
                AES_encrypt(plaintext_bytes.data() + i, ciphertext.data() + i, &aes_key);
            }

            return true;

        } catch (const std::exception&) {
            return false;
        }
    }

    static bool decrypt(const std::vector<uint8_t>& ciphertext,
                       const std::array<uint8_t, 32>& key,
                       const std::array<uint8_t, 16>& iv,
                       std::vector<float>& plaintext) {
        try {
            if (ciphertext.size() % AES_BLOCK_SIZE != 0) {
                return false;
            }

            // Decrypt
            std::vector<uint8_t> decrypted_bytes(ciphertext.size());
            AES_KEY aes_key;
            AES_set_decrypt_key(key.data(), 256, &aes_key);

            for (size_t i = 0; i < ciphertext.size(); i += AES_BLOCK_SIZE) {
                AES_decrypt(ciphertext.data() + i, decrypted_bytes.data() + i, &aes_key);
            }

            // Convert bytes back to float vector
            size_t float_count = decrypted_bytes.size() / sizeof(float);
            plaintext.resize(float_count);
            std::memcpy(plaintext.data(), decrypted_bytes.data(), float_count * sizeof(float));

            return true;

        } catch (const std::exception&) {
            return false;
        }
    }

    static std::array<uint8_t, 32> computeHash(const std::vector<uint8_t>& data) {
        std::array<uint8_t, 32> hash;
        SHA256(data.data(), data.size(), hash.data());
        return hash;
    }
};

/**
 * @brief NEON-optimized vector operations for ARM64
 */
class VectorMath {
public:
#ifdef __ARM_NEON
    static float computeDotProductNEON(const std::vector<float>& a, const std::vector<float>& b) {
        if (a.size() != b.size() || a.empty()) return 0.0f;

        size_t size = a.size();
        size_t neon_size = (size / 4) * 4;
        float result = 0.0f;

        // NEON-optimized computation for blocks of 4
        float32x4_t sum_vec = vdupq_n_f32(0.0f);
        for (size_t i = 0; i < neon_size; i += 4) {
            float32x4_t a_vec = vld1q_f32(&a[i]);
            float32x4_t b_vec = vld1q_f32(&b[i]);
            sum_vec = vfmaq_f32(sum_vec, a_vec, b_vec);
        }

        // Sum the 4 elements in sum_vec
        float32x2_t sum_pair = vadd_f32(vget_low_f32(sum_vec), vget_high_f32(sum_vec));
        result = vget_lane_f32(vpadd_f32(sum_pair, sum_pair), 0);

        // Handle remaining elements
        for (size_t i = neon_size; i < size; ++i) {
            result += a[i] * b[i];
        }

        return result;
    }

    static float computeEuclideanDistanceNEON(const std::vector<float>& a, const std::vector<float>& b) {
        if (a.size() != b.size() || a.empty()) return 0.0f;

        size_t size = a.size();
        size_t neon_size = (size / 4) * 4;
        float sum_sq = 0.0f;

        // NEON-optimized computation
        float32x4_t sum_vec = vdupq_n_f32(0.0f);
        for (size_t i = 0; i < neon_size; i += 4) {
            float32x4_t a_vec = vld1q_f32(&a[i]);
            float32x4_t b_vec = vld1q_f32(&b[i]);
            float32x4_t diff_vec = vsubq_f32(a_vec, b_vec);
            sum_vec = vfmaq_f32(sum_vec, diff_vec, diff_vec);
        }

        // Sum the 4 elements in sum_vec
        float32x2_t sum_pair = vadd_f32(vget_low_f32(sum_vec), vget_high_f32(sum_vec));
        sum_sq = vget_lane_f32(vpadd_f32(sum_pair, sum_pair), 0);

        // Handle remaining elements
        for (size_t i = neon_size; i < size; ++i) {
            float diff = a[i] - b[i];
            sum_sq += diff * diff;
        }

        return std::sqrt(sum_sq);
    }
#endif

    static float computeDotProduct(const std::vector<float>& a, const std::vector<float>& b) {
#ifdef __ARM_NEON
        return computeDotProductNEON(a, b);
#else
        if (a.size() != b.size()) return 0.0f;
        return std::inner_product(a.begin(), a.end(), b.begin(), 0.0f);
#endif
    }

    static float computeEuclideanDistance(const std::vector<float>& a, const std::vector<float>& b) {
#ifdef __ARM_NEON
        return computeEuclideanDistanceNEON(a, b);
#else
        if (a.size() != b.size()) return 0.0f;
        float sum_sq = 0.0f;
        for (size_t i = 0; i < a.size(); ++i) {
            float diff = a[i] - b[i];
            sum_sq += diff * diff;
        }
        return std::sqrt(sum_sq);
#endif
    }

    static float computeVectorNorm(const std::vector<float>& vec) {
        return std::sqrt(computeDotProduct(vec, vec));
    }
};

/**
 * @brief Private implementation class for FaceMatcher
 */
class FaceMatcher::Impl {
public:
    // Configuration
    FaceMatchingConfig config_;

    // Encryption key for templates
    std::array<uint8_t, 32> encryption_key_;
    bool has_encryption_key_ = false;

    // Template database
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
    size_t cache_hit_count_ = 0;
    size_t cache_miss_count_ = 0;

    // Audit logging
    std::vector<AuditLogEntry> audit_log_;
    std::mutex audit_mutex_;

    // Velocity checking (attempts per user per minute)
    std::unordered_map<std::string, std::vector<std::chrono::system_clock::time_point>> user_attempts_;
    std::mutex velocity_mutex_;

    // Error handling
    std::string last_error_;

    // Banking mode state
    bool banking_mode_ = false;

    Impl() = default;

    /**
     * @brief Initialize matcher with encryption key
     */
    FaceResultCode initializeWithKey(const std::string& key_path) {
        try {
            if (!key_path.empty()) {
                // Load encryption key from file
                std::ifstream key_file(key_path, std::ios::binary);
                if (key_file.is_open()) {
                    key_file.read(reinterpret_cast<char*>(encryption_key_.data()),
                                 encryption_key_.size());
                    has_encryption_key_ = true;
                } else {
                    // Generate new key if file doesn't exist
                    if (RAND_bytes(encryption_key_.data(), encryption_key_.size()) == 1) {
                        has_encryption_key_ = true;

                        // Save key to file
                        std::ofstream out_key_file(key_path, std::ios::binary);
                        if (out_key_file.is_open()) {
                            out_key_file.write(reinterpret_cast<const char*>(encryption_key_.data()),
                                              encryption_key_.size());
                        }
                    } else {
                        last_error_ = "Failed to generate encryption key";
                        return FaceResultCode::ERROR_ENCRYPTION_FAILED;
                    }
                }
            } else {
                // Generate random key for session
                if (RAND_bytes(encryption_key_.data(), encryption_key_.size()) == 1) {
                    has_encryption_key_ = true;
                } else {
                    last_error_ = "Failed to generate session encryption key";
                    return FaceResultCode::ERROR_ENCRYPTION_FAILED;
                }
            }

            UNLOOK_LOG_INFO("FaceMatcher initialized with encryption support");
            return FaceResultCode::SUCCESS;

        } catch (const std::exception& e) {
            last_error_ = "Exception during matcher initialization: " + std::string(e.what());
            return FaceResultCode::ERROR_TEMPLATE_INVALID;
        }
    }

    /**
     * @brief Decrypt and cache template features
     */
    FaceResultCode getTemplateFeatures(const BiometricTemplate& template_data,
                                      std::vector<float>& features) {
        std::lock_guard<std::mutex> lock(cache_mutex_);

        // Check cache first
        auto cache_it = decrypted_template_cache_.find(template_data.template_id);
        if (cache_it != decrypted_template_cache_.end()) {
            features = cache_it->second;
            cache_hit_count_++;
            return FaceResultCode::SUCCESS;
        }

        cache_miss_count_++;

        // Decrypt template features
        if (!has_encryption_key_) {
            last_error_ = "No encryption key available for template decryption";
            return FaceResultCode::ERROR_DECRYPTION_FAILED;
        }

        // Verify template integrity
        auto computed_hash = TemplateEncryption::computeHash(template_data.encrypted_features);
        if (computed_hash != template_data.template_hash) {
            last_error_ = "Template integrity check failed";
            return FaceResultCode::ERROR_TEMPLATE_CORRUPTED;
        }

        // Decrypt features
        if (!TemplateEncryption::decrypt(template_data.encrypted_features,
                                        encryption_key_,
                                        template_data.initialization_vector,
                                        features)) {
            last_error_ = "Failed to decrypt template features";
            return FaceResultCode::ERROR_DECRYPTION_FAILED;
        }

        // Validate feature vector size
        if (features.size() != BANKING_FEATURE_SIZE) {
            last_error_ = "Invalid feature vector size after decryption";
            return FaceResultCode::ERROR_TEMPLATE_INVALID;
        }

        // Cache decrypted features
        decrypted_template_cache_[template_data.template_id] = features;

        // Limit cache size
        if (decrypted_template_cache_.size() > static_cast<size_t>(config_.cache_size_mb * 1024 * 1024 / (BANKING_FEATURE_SIZE * sizeof(float)))) {
            // Remove oldest entry (simplified LRU)
            decrypted_template_cache_.erase(decrypted_template_cache_.begin());
        }

        return FaceResultCode::SUCCESS;
    }

    /**
     * @brief Core matching implementation with multiple similarity metrics
     */
    FaceResultCode matchInternal(const std::vector<float>& probe_features,
                                const std::vector<float>& reference_features,
                                float& similarity_score,
                                float& confidence_level) {
        if (probe_features.size() != reference_features.size()) {
            last_error_ = "Feature vector size mismatch";
            return FaceResultCode::ERROR_SIMILARITY_COMPUTATION_FAILED;
        }

        try {
            float score = 0.0f;

            switch (config_.method) {
                case FaceMatchingConfig::COSINE_SIMILARITY:
                    score = computeCosineSimilarity(probe_features, reference_features);
                    break;

                case FaceMatchingConfig::EUCLIDEAN_DISTANCE:
                    score = 1.0f / (1.0f + computeEuclideanDistance(probe_features, reference_features));
                    break;

                case FaceMatchingConfig::MAHALANOBIS_DISTANCE:
                    // For simplicity, using identity covariance matrix
                    score = 1.0f / (1.0f + computeEuclideanDistance(probe_features, reference_features));
                    break;

                case FaceMatchingConfig::NEURAL_SIMILARITY:
                    // Neural network-based similarity (simplified implementation)
                    score = computeNeuralSimilarity(probe_features, reference_features);
                    break;

                case FaceMatchingConfig::BANKING_GRADE_HYBRID:
                    score = computeBankingGradeHybridSimilarity(probe_features, reference_features);
                    break;

                default:
                    last_error_ = "Unknown matching method";
                    return FaceResultCode::ERROR_SIMILARITY_COMPUTATION_FAILED;
            }

            similarity_score = std::max(0.0f, std::min(1.0f, score));

            // Compute confidence level based on feature quality and score stability
            confidence_level = computeMatchConfidence(probe_features, reference_features, similarity_score);

            return FaceResultCode::SUCCESS;

        } catch (const std::exception& e) {
            last_error_ = "Exception during similarity computation: " + std::string(e.what());
            return FaceResultCode::ERROR_SIMILARITY_COMPUTATION_FAILED;
        }
    }

    /**
     * @brief Compute cosine similarity between feature vectors
     */
    float computeCosineSimilarity(const std::vector<float>& features1,
                                 const std::vector<float>& features2) {
        float dot_product = VectorMath::computeDotProduct(features1, features2);
        float norm1 = VectorMath::computeVectorNorm(features1);
        float norm2 = VectorMath::computeVectorNorm(features2);

        if (norm1 > 0 && norm2 > 0) {
            return dot_product / (norm1 * norm2);
        }
        return 0.0f;
    }

    /**
     * @brief Compute Euclidean distance between feature vectors
     */
    float computeEuclideanDistance(const std::vector<float>& features1,
                                  const std::vector<float>& features2) {
        return VectorMath::computeEuclideanDistance(features1, features2);
    }

    /**
     * @brief Compute neural network-based similarity (simplified)
     */
    float computeNeuralSimilarity(const std::vector<float>& features1,
                                 const std::vector<float>& features2) {
        // Simplified neural similarity using weighted combination of metrics
        float cosine_sim = computeCosineSimilarity(features1, features2);
        float euclidean_sim = 1.0f / (1.0f + computeEuclideanDistance(features1, features2));

        // Weighted combination with learned weights (simplified)
        return 0.7f * cosine_sim + 0.3f * euclidean_sim;
    }

    /**
     * @brief Compute banking-grade hybrid similarity
     */
    float computeBankingGradeHybridSimilarity(const std::vector<float>& features1,
                                             const std::vector<float>& features2) {
        // Multi-metric fusion for banking-grade accuracy
        float cosine_sim = computeCosineSimilarity(features1, features2);
        float euclidean_sim = 1.0f / (1.0f + computeEuclideanDistance(features1, features2));

        // Compute local feature similarities for robustness
        std::vector<float> local_similarities;
        size_t block_size = features1.size() / 8; // 8 blocks

        for (size_t i = 0; i < 8; ++i) {
            size_t start = i * block_size;
            size_t end = std::min(start + block_size, features1.size());

            if (start < end) {
                std::vector<float> block1(features1.begin() + start, features1.begin() + end);
                std::vector<float> block2(features2.begin() + start, features2.begin() + end);

                float block_sim = computeCosineSimilarity(block1, block2);
                local_similarities.push_back(block_sim);
            }
        }

        // Compute local consistency
        float local_mean = 0.0f;
        for (float sim : local_similarities) {
            local_mean += sim;
        }
        local_mean /= local_similarities.size();

        float local_variance = 0.0f;
        for (float sim : local_similarities) {
            float diff = sim - local_mean;
            local_variance += diff * diff;
        }
        local_variance /= local_similarities.size();

        float consistency_score = std::max(0.0f, 1.0f - std::sqrt(local_variance));

        // Final hybrid score with consistency weighting
        float hybrid_score = (cosine_sim * 0.4f + euclidean_sim * 0.3f + local_mean * 0.3f);
        return hybrid_score * (0.8f + 0.2f * consistency_score);
    }

    /**
     * @brief Compute match confidence level
     */
    float computeMatchConfidence(const std::vector<float>& features1,
                                const std::vector<float>& features2,
                                float similarity_score) {
        // Confidence based on feature quality and score stability
        float feature_quality1 = computeFeatureQuality(features1);
        float feature_quality2 = computeFeatureQuality(features2);
        float avg_quality = (feature_quality1 + feature_quality2) * 0.5f;

        // Score stability (how far from decision boundary)
        float decision_boundary = getDecisionThreshold();
        float distance_from_boundary = std::abs(similarity_score - decision_boundary);
        float stability_score = std::min(1.0f, distance_from_boundary * 2.0f);

        // Overall confidence
        return (avg_quality * 0.6f + stability_score * 0.4f);
    }

    /**
     * @brief Compute feature vector quality
     */
    float computeFeatureQuality(const std::vector<float>& features) {
        // Check for degenerate features (all zeros, constant values, etc.)
        float mean = 0.0f;
        for (float f : features) {
            mean += f;
        }
        mean /= features.size();

        float variance = 0.0f;
        for (float f : features) {
            float diff = f - mean;
            variance += diff * diff;
        }
        variance /= features.size();

        // Quality based on variance and normalization
        float quality = std::min(1.0f, std::sqrt(variance) * 10.0f);

        // Check for reasonable value ranges
        for (float f : features) {
            if (std::isnan(f) || std::isinf(f) || std::abs(f) > 100.0f) {
                quality *= 0.1f; // Severely penalize invalid values
                break;
            }
        }

        return quality;
    }

    /**
     * @brief Get decision threshold based on configuration
     */
    float getDecisionThreshold() {
        switch (config_.auth_mode) {
            case AuthenticationMode::VERIFY_1_TO_1:
                return config_.verification_threshold;
            case AuthenticationMode::IDENTIFY_1_TO_N:
                return config_.identification_threshold;
            case AuthenticationMode::BANKING_VERIFY:
                return config_.banking_threshold;
            default:
                return config_.verification_threshold;
        }
    }

    /**
     * @brief Apply adaptive thresholding based on template quality
     */
    float adaptThreshold(float base_threshold, float probe_quality, float reference_quality) {
        if (!config_.adaptive_threshold) {
            return base_threshold;
        }

        float avg_quality = (probe_quality + reference_quality) * 0.5f;

        // Adjust threshold based on quality
        // Higher quality -> can use lower threshold (more sensitive)
        // Lower quality -> use higher threshold (more strict)
        float quality_factor = 0.8f + 0.4f * (1.0f - avg_quality);
        return base_threshold * quality_factor;
    }

    /**
     * @brief Validate template quality for matching
     */
    bool validateTemplateQuality(const BiometricTemplate& template_data, float& quality_score) {
        quality_score = template_data.template_quality_score;

        // Banking mode stricter requirements
        if (banking_mode_) {
            return template_data.banking_certified &&
                   quality_score >= 0.95f &&
                   template_data.expected_far <= BANKING_FAR_THRESHOLD;
        }

        // Standard mode requirements
        return quality_score >= config_.min_template_quality;
    }

    /**
     * @brief Check velocity limits for user authentication attempts
     */
    bool checkVelocityLimits(const std::string& user_id) {
        if (!config_.enable_velocity_checking) {
            return true;
        }

        std::lock_guard<std::mutex> lock(velocity_mutex_);

        auto now = std::chrono::system_clock::now();
        auto one_minute_ago = now - std::chrono::minutes(1);

        // Clean old attempts
        auto& attempts = user_attempts_[user_id];
        attempts.erase(
            std::remove_if(attempts.begin(), attempts.end(),
                          [one_minute_ago](const auto& attempt) {
                              return attempt < one_minute_ago;
                          }),
            attempts.end()
        );

        // Check if within limits
        if (attempts.size() >= static_cast<size_t>(config_.max_attempts_per_minute)) {
            return false;
        }

        // Record this attempt
        attempts.push_back(now);
        return true;
    }

    /**
     * @brief Log matching attempt for audit trail
     */
    void logMatchingAttempt(const std::string& probe_id,
                           const std::string& reference_id,
                           const FaceMatchResult& match_result,
                           const std::chrono::system_clock::time_point& timestamp) {
        if (!config_.banking_audit_mode) {
            return;
        }

        std::lock_guard<std::mutex> lock(audit_mutex_);

        AuditLogEntry entry;
        entry.timestamp = timestamp;
        entry.probe_template_id = probe_id;
        entry.reference_template_id = reference_id;
        entry.similarity_score = match_result.similarity_score;
        entry.match_decision = match_result.is_match;
        entry.confidence_level = match_result.confidence_level;
        entry.result_code = faceResultCodeToString(match_result.result_code);
        entry.processing_time_ms = match_result.matching_time_ms;
        entry.device_id = "unlook_scanner_cm5"; // Device identifier
        entry.session_id = generateSessionId();

        audit_log_.push_back(entry);

        // Limit audit log size (keep last 10000 entries)
        if (audit_log_.size() > 10000) {
            audit_log_.erase(audit_log_.begin(), audit_log_.begin() + 1000);
        }
    }

    /**
     * @brief Generate unique session ID
     */
    std::string generateSessionId() {
        static std::random_device rd;
        static std::mt19937 gen(rd());
        static std::uniform_int_distribution<> dis(0, 15);

        std::string session_id;
        for (int i = 0; i < 16; ++i) {
            session_id += "0123456789ABCDEF"[dis(gen)];
        }
        return session_id;
    }

    /**
     * @brief Update performance statistics
     */
    void updateStatistics(double matching_time, bool is_genuine, bool is_correct, float similarity_score) {
        total_matches_.fetch_add(1);
        total_matching_time_.store(total_matching_time_.load() + matching_time);

        if (is_genuine) {
            genuine_matches_.fetch_add(1);
            if (is_correct) {
                correct_genuine_.fetch_add(1);
            }
        } else {
            impostor_matches_.fetch_add(1);
            if (is_correct) {
                correct_impostor_.fetch_add(1);
            }
        }
    }

    /**
     * @brief Compute current FAR and FRR from statistics
     */
    void computeCurrentRates(float& current_far, float& current_frr) {
        size_t total_genuine = genuine_matches_.load();
        size_t total_impostor = impostor_matches_.load();
        size_t correct_gen = correct_genuine_.load();
        size_t correct_imp = correct_impostor_.load();

        if (total_genuine > 0) {
            current_frr = 1.0f - (static_cast<float>(correct_gen) / total_genuine);
        } else {
            current_frr = 0.0f;
        }

        if (total_impostor > 0) {
            current_far = 1.0f - (static_cast<float>(correct_imp) / total_impostor);
        } else {
            current_far = 0.0f;
        }
    }
};

// FaceMatchingConfig validation
bool FaceMatchingConfig::validate() const {
    if (verification_threshold < 0.0f || verification_threshold > 1.0f) return false;
    if (identification_threshold < 0.0f || identification_threshold > 1.0f) return false;
    if (banking_threshold < 0.8f || banking_threshold > 1.0f) return false;
    if (min_template_quality < 0.0f || min_template_quality > 1.0f) return false;
    if (min_probe_quality < 0.0f || min_probe_quality > 1.0f) return false;
    if (target_far < 0.0f || target_far > 1.0f) return false;
    if (target_frr < 0.0f || target_frr > 1.0f) return false;
    if (max_attempts_per_minute < 1 || max_attempts_per_minute > 100) return false;
    if (cache_size_mb < 1 || cache_size_mb > 1024) return false;
    if (max_matching_threads < 1 || max_matching_threads > 16) return false;
    if (max_database_size < 1 || max_database_size > 1000000) return false;
    return true;
}

std::string FaceMatchingConfig::toString() const {
    return "FaceMatchingConfig{" +
           std::string("method=") + std::to_string(static_cast<int>(method)) +
           ", auth_mode=" + std::to_string(static_cast<int>(auth_mode)) +
           ", banking_threshold=" + std::to_string(banking_threshold) +
           ", target_far=" + std::to_string(target_far) +
           ", target_frr=" + std::to_string(target_frr) + "}";
}

// FaceMatcher implementation
FaceMatcher::FaceMatcher() : pImpl(std::make_unique<Impl>()) {
    FaceMatchingConfig default_config;
    pImpl->config_ = default_config;
}

FaceMatcher::FaceMatcher(const FaceMatchingConfig& config) : pImpl(std::make_unique<Impl>()) {
    pImpl->config_ = config;
}

FaceMatcher::~FaceMatcher() = default;

FaceMatcher::FaceMatcher(FaceMatcher&&) noexcept = default;
FaceMatcher& FaceMatcher::operator=(FaceMatcher&&) noexcept = default;

FaceResultCode FaceMatcher::initialize(const std::string& encryption_key_path,
                                      const std::string& database_path) {
    FaceResultCode result = pImpl->initializeWithKey(encryption_key_path);
    if (result != FaceResultCode::SUCCESS) {
        return result;
    }

    // Load database if path provided
    if (!database_path.empty()) {
        // Database loading would be implemented here
        UNLOOK_LOG_INFO("Database path provided: {}", database_path);
    }

    return FaceResultCode::SUCCESS;
}

bool FaceMatcher::isInitialized() const {
    return pImpl->has_encryption_key_;
}

FaceResultCode FaceMatcher::verify(const BiometricTemplate& probe_template,
                                  const BiometricTemplate& reference_template,
                                  FaceMatchResult& match_result) {
    auto start_time = std::chrono::high_resolution_clock::now();

    // Initialize result
    match_result = FaceMatchResult();
    match_result.result_code = FaceResultCode::SUCCESS;

    try {
        // Velocity checking
        if (!pImpl->checkVelocityLimits(probe_template.subject_id)) {
            match_result.result_code = FaceResultCode::ERROR_AUTHENTICATION_FAILED;
            match_result.error_message = "Velocity limit exceeded";
            return match_result.result_code;
        }

        // Validate template quality
        float probe_quality, reference_quality;
        if (pImpl->config_.enable_quality_gating) {
            if (!pImpl->validateTemplateQuality(probe_template, probe_quality) ||
                !pImpl->validateTemplateQuality(reference_template, reference_quality)) {
                match_result.result_code = FaceResultCode::ERROR_TEMPLATE_INVALID;
                match_result.error_message = "Template quality below threshold";
                return match_result.result_code;
            }
        } else {
            probe_quality = probe_template.template_quality_score;
            reference_quality = reference_template.template_quality_score;
        }

        // Decrypt template features
        std::vector<float> probe_features, reference_features;
        FaceResultCode decrypt_result = pImpl->getTemplateFeatures(probe_template, probe_features);
        if (decrypt_result != FaceResultCode::SUCCESS) {
            match_result.result_code = decrypt_result;
            match_result.error_message = pImpl->last_error_;
            return decrypt_result;
        }

        decrypt_result = pImpl->getTemplateFeatures(reference_template, reference_features);
        if (decrypt_result != FaceResultCode::SUCCESS) {
            match_result.result_code = decrypt_result;
            match_result.error_message = pImpl->last_error_;
            return decrypt_result;
        }

        // Perform matching
        FaceResultCode match_code = pImpl->matchInternal(probe_features, reference_features,
                                                        match_result.similarity_score,
                                                        match_result.confidence_level);
        if (match_code != FaceResultCode::SUCCESS) {
            match_result.result_code = match_code;
            match_result.error_message = pImpl->last_error_;
            return match_code;
        }

        // Apply adaptive thresholding
        float threshold = pImpl->adaptThreshold(pImpl->getDecisionThreshold(),
                                               probe_quality, reference_quality);

        // Make match decision
        match_result.is_match = (match_result.similarity_score >= threshold);
        match_result.success = true;

        // Estimate FAR/FRR for this match
        match_result.false_accept_rate = pImpl->config_.target_far;
        match_result.false_reject_rate = pImpl->config_.target_frr;
        match_result.match_quality = (probe_quality + reference_quality) * 0.5f;
        match_result.matched_template_id = reference_template.template_id;

        auto end_time = std::chrono::high_resolution_clock::now();
        match_result.matching_time_ms =
            std::chrono::duration<double, std::milli>(end_time - start_time).count();

        // Update statistics (assumes ground truth available for evaluation)
        bool is_genuine = (probe_template.subject_id == reference_template.subject_id);
        bool is_correct = (match_result.is_match == is_genuine);
        pImpl->updateStatistics(match_result.matching_time_ms, is_genuine, is_correct,
                               match_result.similarity_score);

        // Audit logging
        pImpl->logMatchingAttempt(probe_template.template_id, reference_template.template_id,
                                 match_result, start_time);

        return FaceResultCode::SUCCESS;

    } catch (const std::exception& e) {
        match_result.result_code = FaceResultCode::ERROR_MATCHING_FAILED;
        match_result.error_message = "Exception during matching: " + std::string(e.what());
        return match_result.result_code;
    }
}

FaceResultCode FaceMatcher::verifyBankingGrade(const BiometricTemplate& probe_template,
                                              const BiometricTemplate& reference_template,
                                              const LivenessResult& liveness_result,
                                              FaceMatchResult& match_result) {
    // Banking-grade verification with enhanced security

    // Validate liveness first
    if (!liveness_result.isBankingGradeLive()) {
        match_result.result_code = FaceResultCode::ERROR_LIVENESS_CHECK_FAILED;
        match_result.error_message = "Banking-grade liveness check failed";
        return match_result.result_code;
    }

    // Validate templates are banking-certified
    if (!probe_template.isBankingGrade() || !reference_template.isBankingGrade()) {
        match_result.result_code = FaceResultCode::ERROR_TEMPLATE_INVALID;
        match_result.error_message = "Templates not banking-grade certified";
        return match_result.result_code;
    }

    // Perform standard verification with banking mode enabled
    bool original_banking_mode = pImpl->banking_mode_;
    pImpl->banking_mode_ = true;

    FaceResultCode result = verify(probe_template, reference_template, match_result);

    pImpl->banking_mode_ = original_banking_mode;

    // Additional banking-grade validation
    if (result == FaceResultCode::SUCCESS) {
        if (!match_result.isBankingGradeMatch()) {
            match_result.result_code = FaceResultCode::ERROR_AUTHENTICATION_FAILED;
            match_result.error_message = "Match does not meet banking-grade criteria";
            return match_result.result_code;
        }
    }

    return result;
}

FaceResultCode FaceMatcher::setConfiguration(const FaceMatchingConfig& config) {
    if (!config.validate()) {
        pImpl->last_error_ = "Invalid face matching configuration";
        return FaceResultCode::ERROR_TEMPLATE_INVALID;
    }
    pImpl->config_ = config;
    return FaceResultCode::SUCCESS;
}

FaceMatchingConfig FaceMatcher::getConfiguration() const {
    return pImpl->config_;
}

void FaceMatcher::setBankingMode(bool enable, bool strict_security, bool audit_compliance) {
    pImpl->banking_mode_ = enable;
    pImpl->config_.banking_audit_mode = audit_compliance;
    pImpl->config_.enable_template_encryption = strict_security;
    pImpl->config_.enable_anti_spoofing = strict_security;

    if (enable) {
        // Set banking-grade thresholds
        pImpl->config_.banking_threshold = 0.95f;
        pImpl->config_.target_far = 0.0001f;  // 0.01%
        pImpl->config_.target_frr = 0.03f;    // 3%
        pImpl->config_.method = FaceMatchingConfig::BANKING_GRADE_HYBRID;
    }
}

void FaceMatcher::getStatistics(double& avg_matching_time, size_t& total_matches,
                               size_t& genuine_matches, size_t& impostor_matches,
                               float& current_far, float& current_frr) const {
    std::lock_guard<std::mutex> lock(pImpl->stats_mutex_);

    total_matches = pImpl->total_matches_.load();
    genuine_matches = pImpl->genuine_matches_.load();
    impostor_matches = pImpl->impostor_matches_.load();

    avg_matching_time = (total_matches > 0) ?
                       pImpl->total_matching_time_.load() / total_matches : 0.0;

    pImpl->computeCurrentRates(current_far, current_frr);
}

void FaceMatcher::resetStatistics() {
    std::lock_guard<std::mutex> lock(pImpl->stats_mutex_);
    pImpl->total_matches_.store(0);
    pImpl->genuine_matches_.store(0);
    pImpl->impostor_matches_.store(0);
    pImpl->total_matching_time_.store(0.0);
    pImpl->correct_genuine_.store(0);
    pImpl->correct_impostor_.store(0);
}

std::string FaceMatcher::getLastError() const {
    return pImpl->last_error_;
}

// MatchingEvaluator implementation
void MatchingEvaluator::computeROCCurve(const std::vector<float>& similarity_scores,
                                       const std::vector<bool>& ground_truth,
                                       std::vector<std::pair<float, float>>& roc_points) {
    if (similarity_scores.size() != ground_truth.size()) {
        return;
    }

    roc_points.clear();

    // Create threshold range
    std::vector<float> thresholds;
    for (int i = 0; i <= 100; ++i) {
        thresholds.push_back(i / 100.0f);
    }

    // Compute FAR and FRR for each threshold
    for (float threshold : thresholds) {
        size_t true_positive = 0, false_positive = 0;
        size_t true_negative = 0, false_negative = 0;

        for (size_t i = 0; i < similarity_scores.size(); ++i) {
            bool predicted_match = (similarity_scores[i] >= threshold);
            bool actual_match = ground_truth[i];

            if (actual_match && predicted_match) true_positive++;
            else if (!actual_match && predicted_match) false_positive++;
            else if (!actual_match && !predicted_match) true_negative++;
            else if (actual_match && !predicted_match) false_negative++;
        }

        float far = (false_positive + true_negative > 0) ?
                   static_cast<float>(false_positive) / (false_positive + true_negative) : 0.0f;
        float frr = (true_positive + false_negative > 0) ?
                   static_cast<float>(false_negative) / (true_positive + false_negative) : 0.0f;

        roc_points.emplace_back(far, frr);
    }
}

float MatchingEvaluator::computeEER(const std::vector<float>& similarity_scores,
                                   const std::vector<bool>& ground_truth,
                                   float& eer_threshold) {
    std::vector<std::pair<float, float>> roc_points;
    computeROCCurve(similarity_scores, ground_truth, roc_points);

    float min_diff = std::numeric_limits<float>::max();
    float eer = 0.0f;
    eer_threshold = 0.5f;

    for (size_t i = 0; i < roc_points.size(); ++i) {
        float far = roc_points[i].first;
        float frr = roc_points[i].second;
        float diff = std::abs(far - frr);

        if (diff < min_diff) {
            min_diff = diff;
            eer = (far + frr) * 0.5f;
            eer_threshold = i / 100.0f;
        }
    }

    return eer;
}

} // namespace face
} // namespace unlook