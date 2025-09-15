#pragma once

#include "unlook/face/FaceTypes.hpp"
#include <opencv2/opencv.hpp>
#include <memory>
#include <vector>
#include <array>
#include <string>
#include <mutex>
#include <atomic>

namespace unlook {
namespace face {

/**
 * @brief Face encoding configuration for template generation
 */
struct FaceEncodingConfig {
    // Encoding method
    enum Method {
        DEEP_FEATURES,          ///< Deep neural network features
        GEOMETRIC_FEATURES,     ///< Geometric landmark features
        HYBRID_FEATURES,        ///< Hybrid deep + geometric features
        BANKING_GRADE          ///< Banking-grade multi-modal encoding
    } method = BANKING_GRADE;

    // Feature extraction parameters
    int feature_vector_size = 2048;     ///< Size of feature vector
    std::string model_path;             ///< Path to encoding model
    bool normalize_features = true;     ///< L2-normalize feature vectors
    bool enable_pca_reduction = false;  ///< Apply PCA dimensionality reduction
    int pca_components = 512;           ///< Number of PCA components

    // Geometric feature parameters
    bool include_landmark_features = true;  ///< Include landmark-based features
    bool include_geometric_ratios = true;   ///< Include facial ratio features
    bool include_texture_features = true;   ///< Include texture-based features
    bool include_3d_features = true;        ///< Include 3D shape features

    // Quality and robustness
    float min_feature_quality = 0.8f;      ///< Minimum feature extraction quality
    bool enable_pose_invariance = true;    ///< Enable pose-invariant features
    bool enable_illumination_normalization = true; ///< Normalize illumination
    int augmentation_samples = 5;          ///< Number of augmented samples

    // Security and encryption
    bool enable_encryption = true;         ///< Enable template encryption
    std::string encryption_key_path;       ///< Path to encryption key
    bool enable_template_hashing = true;   ///< Enable template integrity hashing
    bool enable_biometric_salting = true;  ///< Enable biometric salting

    // Banking compliance
    bool iso_compliance_mode = false;      ///< ISO/IEC 19794-5 compliance
    bool banking_certification_mode = false; ///< Banking certification mode
    float banking_quality_threshold = 0.95f; ///< Banking quality requirement
    bool require_liveness_proof = true;    ///< Require liveness verification

    // Performance settings
    bool enable_gpu_acceleration = false;  ///< Use GPU acceleration
    bool enable_multithreading = true;     ///< Multi-threaded processing
    int max_threads = 2;                   ///< Maximum encoding threads

    bool validate() const;
    std::string toString() const;
};

/**
 * @brief Banking-grade facial feature encoder and template generator
 *
 * Advanced facial encoding system that generates secure, encrypted biometric
 * templates suitable for banking-grade authentication systems. Combines
 * deep learning features with geometric measurements and 3D shape data.
 *
 * Key features:
 * - Multi-modal feature extraction (deep + geometric + 3D)
 * - AES-256 template encryption with biometric salting
 * - ISO/IEC 19794-5 compliance for banking applications
 * - Pose-invariant and illumination-robust features
 * - Template integrity verification with cryptographic hashing
 * - Supernova ML integration interface
 * - ARM64/NEON optimized for CM4/CM5 performance
 *
 * Designed for FAR < 0.001% and FRR < 3% banking requirements.
 */
class FaceEncoder {
public:
    /**
     * @brief Constructor with default configuration
     */
    FaceEncoder();

    /**
     * @brief Constructor with custom configuration
     * @param config Encoding configuration
     */
    explicit FaceEncoder(const FaceEncodingConfig& config);

    /**
     * @brief Destructor
     */
    ~FaceEncoder();

    /**
     * @brief Initialize encoder with models and keys
     * @param model_path Path to feature extraction model
     * @param encryption_key_path Path to encryption key file
     * @return FaceResultCode indicating success or failure
     */
    FaceResultCode initialize(const std::string& model_path,
                             const std::string& encryption_key_path = "");

    /**
     * @brief Check if encoder is initialized
     * @return True if encoder is ready for use
     */
    bool isInitialized() const;

    /**
     * @brief Generate biometric template from face data
     * @param face_image Normalized face image (CV_8UC3)
     * @param landmarks 3D facial landmarks
     * @param face_model Optional 3D face model
     * @param template_result Output encrypted biometric template
     * @return FaceResultCode indicating success or failure
     */
    FaceResultCode generateTemplate(const cv::Mat& face_image,
                                   const FacialLandmarks& landmarks,
                                   const Face3DModel* face_model,
                                   BiometricTemplate& template_result);

    /**
     * @brief Generate template from multiple face samples (enrollment)
     * @param face_images Vector of face images
     * @param landmarks_vector Vector of corresponding landmarks
     * @param face_models Optional vector of 3D face models
     * @param template_result Output averaged encrypted template
     * @return FaceResultCode indicating success or failure
     */
    FaceResultCode generateTemplateFromSamples(
        const std::vector<cv::Mat>& face_images,
        const std::vector<FacialLandmarks>& landmarks_vector,
        const std::vector<Face3DModel>* face_models,
        BiometricTemplate& template_result);

    /**
     * @brief Asynchronous template generation
     * @param face_image Input face image
     * @param landmarks Facial landmarks
     * @param face_model Optional 3D face model
     * @param callback Callback function for result
     * @return True if encoding task was queued successfully
     */
    bool generateTemplateAsync(const cv::Mat& face_image,
                              const FacialLandmarks& landmarks,
                              const Face3DModel* face_model,
                              TemplateGenerationCallback callback);

    /**
     * @brief Extract raw feature vector (unencrypted)
     * @param face_image Input face image
     * @param landmarks Facial landmarks
     * @param feature_vector Output feature vector
     * @return FaceResultCode indicating success or failure
     */
    FaceResultCode extractFeatures(const cv::Mat& face_image,
                                  const FacialLandmarks& landmarks,
                                  std::vector<float>& feature_vector);

    /**
     * @brief Validate template integrity
     * @param template_data Input biometric template
     * @param is_valid Output validity flag
     * @param error_message Output error description
     * @return FaceResultCode indicating validation result
     */
    FaceResultCode validateTemplate(const BiometricTemplate& template_data,
                                   bool& is_valid,
                                   std::string& error_message);

    /**
     * @brief Update template with additional samples
     * @param existing_template Input/output template
     * @param new_face_image New face sample
     * @param new_landmarks New landmarks
     * @param new_face_model Optional new 3D model
     * @return FaceResultCode indicating success or failure
     */
    FaceResultCode updateTemplate(BiometricTemplate& existing_template,
                                 const cv::Mat& new_face_image,
                                 const FacialLandmarks& new_landmarks,
                                 const Face3DModel* new_face_model = nullptr);

    /**
     * @brief Decrypt template for matching (internal use)
     * @param encrypted_template Input encrypted template
     * @param encryption_key Decryption key
     * @param raw_features Output raw feature vector
     * @return FaceResultCode indicating success or failure
     */
    FaceResultCode decryptTemplate(const BiometricTemplate& encrypted_template,
                                  const std::array<uint8_t, 32>& encryption_key,
                                  std::vector<float>& raw_features);

    /**
     * @brief Convert template to Supernova ML format
     * @param unlook_template Input Unlook template
     * @param supernova_data Output Supernova-compatible data
     * @param metadata Output metadata for ML system
     * @return FaceResultCode indicating success or failure
     */
    FaceResultCode convertToSupernovaFormat(
        const BiometricTemplate& unlook_template,
        std::vector<uint8_t>& supernova_data,
        std::map<std::string, std::string>& metadata);

    /**
     * @brief Set encoding configuration
     * @param config New configuration
     * @return FaceResultCode indicating success or failure
     */
    FaceResultCode setConfiguration(const FaceEncodingConfig& config);

    /**
     * @brief Get current configuration
     * @return Current encoding configuration
     */
    FaceEncodingConfig getConfiguration() const;

    /**
     * @brief Set encryption key from binary data
     * @param key_data 256-bit encryption key
     * @return FaceResultCode indicating success or failure
     */
    FaceResultCode setEncryptionKey(const std::array<uint8_t, 32>& key_data);

    /**
     * @brief Generate new encryption key
     * @param key_data Output generated 256-bit key
     * @return FaceResultCode indicating success or failure
     */
    FaceResultCode generateEncryptionKey(std::array<uint8_t, 32>& key_data);

    /**
     * @brief Enable/disable specific encoding features
     * @param enable_deep Enable deep learning features
     * @param enable_geometric Enable geometric features
     * @param enable_3d Enable 3D shape features
     * @param enable_texture Enable texture features
     */
    void setFeatures(bool enable_deep,
                    bool enable_geometric,
                    bool enable_3d,
                    bool enable_texture);

    /**
     * @brief Get encoding statistics
     * @param avg_encoding_time Average encoding time in milliseconds
     * @param total_encodings Total number of encodings performed
     * @param success_rate Success rate (0-1)
     * @param avg_template_size Average template size in bytes
     */
    void getStatistics(double& avg_encoding_time,
                      size_t& total_encodings,
                      float& success_rate,
                      size_t& avg_template_size) const;

    /**
     * @brief Reset encoding statistics
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
     * @brief Get supported encoding methods
     * @return Vector of supported encoding method names
     */
    static std::vector<std::string> getSupportedMethods();

    /**
     * @brief Validate encoding model
     * @param model_path Path to model file
     * @param method Expected encoding method
     * @return True if model is valid and compatible
     */
    static bool validateModel(const std::string& model_path,
                             FaceEncodingConfig::Method method);

    /**
     * @brief Get recommended feature vector size for method
     * @param method Encoding method
     * @return Recommended feature vector size
     */
    static int getRecommendedFeatureSize(FaceEncodingConfig::Method method);

    /**
     * @brief Enable banking-grade encoding mode
     * @param enable Enable/disable banking mode
     * @param iso_compliant Require ISO compliance
     * @param min_quality Minimum template quality
     */
    void setBankingMode(bool enable,
                       bool iso_compliant = true,
                       float min_quality = 0.95f);

private:
    class Impl;
    std::unique_ptr<Impl> pImpl;

    // Performance monitoring
    mutable std::mutex stats_mutex_;
    std::atomic<size_t> total_encodings_{0};
    std::atomic<size_t> successful_encodings_{0};
    std::atomic<double> total_encoding_time_{0.0};
    std::atomic<size_t> total_template_size_{0};

    // Encryption state
    std::mutex crypto_mutex_;
    std::array<uint8_t, 32> encryption_key_;
    bool has_encryption_key_{false};

    /**
     * @brief Internal template generation implementation
     * @param face_images Input face images
     * @param landmarks_vector Input landmarks
     * @param face_models Input 3D models (optional)
     * @param template_result Output template
     * @return FaceResultCode
     */
    FaceResultCode generateTemplateInternal(
        const std::vector<cv::Mat>& face_images,
        const std::vector<FacialLandmarks>& landmarks_vector,
        const std::vector<Face3DModel>* face_models,
        BiometricTemplate& template_result);

    /**
     * @brief Extract deep learning features
     * @param face_image Input face image
     * @param deep_features Output deep features
     * @return FaceResultCode
     */
    FaceResultCode extractDeepFeatures(const cv::Mat& face_image,
                                      std::vector<float>& deep_features);

    /**
     * @brief Extract geometric features from landmarks
     * @param landmarks Input facial landmarks
     * @param geometric_features Output geometric features
     * @return FaceResultCode
     */
    FaceResultCode extractGeometricFeatures(const FacialLandmarks& landmarks,
                                           std::vector<float>& geometric_features);

    /**
     * @brief Extract 3D shape features
     * @param face_model Input 3D face model
     * @param shape_features Output 3D shape features
     * @return FaceResultCode
     */
    FaceResultCode extract3DFeatures(const Face3DModel& face_model,
                                    std::vector<float>& shape_features);

    /**
     * @brief Extract texture features
     * @param face_image Input face image
     * @param landmarks Facial landmarks for region extraction
     * @param texture_features Output texture features
     * @return FaceResultCode
     */
    FaceResultCode extractTextureFeatures(const cv::Mat& face_image,
                                         const FacialLandmarks& landmarks,
                                         std::vector<float>& texture_features);

    /**
     * @brief Combine multiple feature vectors
     * @param feature_vectors Input feature vectors
     * @param weights Feature weights
     * @param combined_features Output combined features
     * @return FaceResultCode
     */
    FaceResultCode combineFeatures(const std::vector<std::vector<float>>& feature_vectors,
                                  const std::vector<float>& weights,
                                  std::vector<float>& combined_features);

    /**
     * @brief Apply feature normalization and post-processing
     * @param raw_features Input raw features
     * @param normalized_features Output normalized features
     */
    void normalizeFeatures(const std::vector<float>& raw_features,
                          std::vector<float>& normalized_features);

    /**
     * @brief Encrypt feature vector using AES-256
     * @param raw_features Input raw features
     * @param encryption_key 256-bit encryption key
     * @param encrypted_data Output encrypted data
     * @param initialization_vector Output IV
     * @return FaceResultCode
     */
    FaceResultCode encryptFeatures(const std::vector<float>& raw_features,
                                  const std::array<uint8_t, 32>& encryption_key,
                                  std::vector<uint8_t>& encrypted_data,
                                  std::array<uint8_t, 16>& initialization_vector);

    /**
     * @brief Decrypt feature vector using AES-256
     * @param encrypted_data Input encrypted data
     * @param encryption_key 256-bit encryption key
     * @param initialization_vector IV used for encryption
     * @param raw_features Output decrypted features
     * @return FaceResultCode
     */
    FaceResultCode decryptFeatures(const std::vector<uint8_t>& encrypted_data,
                                  const std::array<uint8_t, 32>& encryption_key,
                                  const std::array<uint8_t, 16>& initialization_vector,
                                  std::vector<float>& raw_features);

    /**
     * @brief Compute cryptographic hash of template
     * @param template_data Input template data
     * @param hash_output Output SHA-256 hash
     */
    void computeTemplateHash(const BiometricTemplate& template_data,
                            std::array<uint8_t, 32>& hash_output);

    /**
     * @brief Apply biometric salting for template security
     * @param raw_features Input features
     * @param salt_value Biometric-derived salt
     * @param salted_features Output salted features
     */
    void applyBiometricSalting(const std::vector<float>& raw_features,
                              const std::array<uint8_t, 16>& salt_value,
                              std::vector<float>& salted_features);

    /**
     * @brief Assess template quality for banking compliance
     * @param template_data Input template
     * @param quality_score Output quality score (0-1)
     * @return Quality assessment details
     */
    std::map<std::string, float> assessTemplateQuality(
        const BiometricTemplate& template_data,
        float& quality_score);

    /**
     * @brief Update performance statistics
     * @param encoding_time Time taken for encoding
     * @param success Whether encoding was successful
     * @param template_size Template size in bytes
     */
    void updateStatistics(double encoding_time,
                         bool success,
                         size_t template_size);

    // Disable copy constructor and assignment
    FaceEncoder(const FaceEncoder&) = delete;
    FaceEncoder& operator=(const FaceEncoder&) = delete;

    // Enable move semantics
    FaceEncoder(FaceEncoder&&) noexcept;
    FaceEncoder& operator=(FaceEncoder&&) noexcept;
};

/**
 * @brief Cryptographic utilities for secure template handling
 */
class TemplateCrypto {
public:
    /**
     * @brief Generate secure random key
     * @param key_data Output 256-bit key
     * @return True if key generated successfully
     */
    static bool generateSecureKey(std::array<uint8_t, 32>& key_data);

    /**
     * @brief Derive key from password using PBKDF2
     * @param password Input password
     * @param salt Salt value
     * @param iterations Number of iterations
     * @param key_data Output derived key
     * @return True if derivation successful
     */
    static bool deriveKeyFromPassword(const std::string& password,
                                     const std::array<uint8_t, 16>& salt,
                                     int iterations,
                                     std::array<uint8_t, 32>& key_data);

    /**
     * @brief Compute SHA-256 hash
     * @param data Input data
     * @param hash_output Output hash
     */
    static void computeSHA256(const std::vector<uint8_t>& data,
                             std::array<uint8_t, 32>& hash_output);

    /**
     * @brief Generate secure random IV
     * @param iv_data Output 128-bit IV
     * @return True if IV generated successfully
     */
    static bool generateSecureIV(std::array<uint8_t, 16>& iv_data);

    /**
     * @brief Validate encryption key strength
     * @param key_data Input key to validate
     * @return True if key meets security requirements
     */
    static bool validateKeyStrength(const std::array<uint8_t, 32>& key_data);
};

} // namespace face
} // namespace unlook