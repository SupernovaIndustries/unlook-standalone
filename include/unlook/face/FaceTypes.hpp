#pragma once

#include "unlook/core/types.hpp"
#include <opencv2/opencv.hpp>
#include <memory>
#include <vector>
#include <functional>
#include <chrono>
#include <string>
#include <array>

namespace unlook {
namespace face {

/**
 * @brief Facial recognition API types and structures for banking-grade authentication
 *
 * This header defines all core types used throughout the facial recognition system,
 * designed for industrial-grade precision and banking-level security compliance.
 */

// Forward declarations
struct FaceDetectionResult;
struct FacialLandmarks;
struct Face3DModel;
struct BiometricTemplate;
struct FaceMatchResult;
struct LivenessResult;

/// Face-specific result codes extending core ResultCode
enum class FaceResultCode : int32_t {
    SUCCESS = 0,

    // Detection errors
    ERROR_NO_FACE_DETECTED = -100,
    ERROR_MULTIPLE_FACES_DETECTED = -101,
    ERROR_FACE_TOO_SMALL = -102,
    ERROR_FACE_TOO_LARGE = -103,
    ERROR_FACE_OUT_OF_BOUNDS = -104,
    ERROR_POOR_FACE_QUALITY = -105,

    // Landmark errors
    ERROR_LANDMARK_DETECTION_FAILED = -110,
    ERROR_INSUFFICIENT_LANDMARKS = -111,
    ERROR_LANDMARK_QUALITY_LOW = -112,

    // 3D reconstruction errors
    ERROR_DEPTH_DATA_INVALID = -120,
    ERROR_STEREO_RECONSTRUCTION_FAILED = -121,
    ERROR_3D_MODEL_INCOMPLETE = -122,
    ERROR_MESH_GENERATION_FAILED = -123,

    // Template errors
    ERROR_TEMPLATE_GENERATION_FAILED = -130,
    ERROR_TEMPLATE_INVALID = -131,
    ERROR_TEMPLATE_CORRUPTED = -132,
    ERROR_TEMPLATE_VERSION_MISMATCH = -133,

    // Matching errors
    ERROR_MATCHING_FAILED = -140,
    ERROR_TEMPLATE_DATABASE_ERROR = -141,
    ERROR_SIMILARITY_COMPUTATION_FAILED = -142,

    // Liveness errors
    ERROR_LIVENESS_CHECK_FAILED = -150,
    ERROR_PRESENTATION_ATTACK_DETECTED = -151,
    ERROR_DEPTH_LIVENESS_FAILED = -152,
    ERROR_MOTION_LIVENESS_FAILED = -153,

    // Security errors
    ERROR_ENCRYPTION_FAILED = -160,
    ERROR_DECRYPTION_FAILED = -161,
    ERROR_AUTHENTICATION_FAILED = -162,
    ERROR_AUTHORIZATION_DENIED = -163,

    // Hardware/Integration errors
    ERROR_CAMERA_NOT_AVAILABLE = -170,
    ERROR_DEPTH_SENSOR_FAILED = -171,
    ERROR_LED_CONTROL_FAILED = -172,
    ERROR_SUPERNOVA_ML_ERROR = -173
};

/// Face detection quality levels
enum class FaceQuality {
    LOW = 0,        ///< Basic quality, suitable for detection only
    MEDIUM = 1,     ///< Good quality, suitable for recognition
    HIGH = 2,       ///< High quality, suitable for enrollment
    BANKING = 3     ///< Banking grade quality, highest precision
};

/// Facial landmark models supported
enum class LandmarkModel {
    LANDMARKS_68,   ///< 68-point facial landmarks (dlib standard)
    LANDMARKS_468,  ///< 468-point facial landmarks (MediaPipe style)
    LANDMARKS_3D_68 ///< 68-point landmarks with 3D coordinates
};

/// Liveness detection methods
enum class LivenessMethod {
    PASSIVE_DEPTH,      ///< Depth-based liveness using stereo data
    ACTIVE_MOTION,      ///< Motion-based liveness detection
    MULTI_MODAL,        ///< Combined depth + motion + texture
    BANKING_GRADE       ///< Full banking compliance liveness
};

/// Face enrollment states
enum class EnrollmentState {
    NOT_STARTED,
    POSITIONING,        ///< User positioning for optimal capture
    CAPTURING,          ///< Capturing multiple face samples
    PROCESSING,         ///< Processing captured data
    QUALITY_CHECK,      ///< Validating enrollment quality
    COMPLETED,          ///< Enrollment successfully completed
    FAILED              ///< Enrollment failed
};

/// Authentication modes
enum class AuthenticationMode {
    VERIFY_1_TO_1,      ///< 1:1 verification against provided template
    IDENTIFY_1_TO_N,    ///< 1:N identification in database
    BANKING_VERIFY      ///< Banking-grade 1:1 with enhanced security
};

/**
 * @brief 2D facial landmark point with confidence
 */
struct LandmarkPoint2D {
    float x, y;           ///< 2D coordinates in image space
    float confidence;     ///< Detection confidence (0-1)

    LandmarkPoint2D() : x(0), y(0), confidence(0) {}
    LandmarkPoint2D(float x_, float y_, float conf = 1.0f)
        : x(x_), y(y_), confidence(conf) {}
};

/**
 * @brief 3D facial landmark point with depth and confidence
 */
struct LandmarkPoint3D {
    float x, y, z;        ///< 3D coordinates in camera space (mm)
    float confidence;     ///< Detection confidence (0-1)
    float depth_quality;  ///< Quality of depth measurement (0-1)

    LandmarkPoint3D() : x(0), y(0), z(0), confidence(0), depth_quality(0) {}
    LandmarkPoint3D(float x_, float y_, float z_, float conf = 1.0f, float dq = 1.0f)
        : x(x_), y(y_), z(z_), confidence(conf), depth_quality(dq) {}
};

/**
 * @brief Facial landmarks container with validation
 */
struct FacialLandmarks {
    LandmarkModel model;                          ///< Landmark model type
    std::vector<LandmarkPoint2D> points2D;       ///< 2D landmark points
    std::vector<LandmarkPoint3D> points3D;       ///< 3D landmark points (if available)

    // Quality metrics
    float overall_confidence;     ///< Overall landmark confidence
    float geometric_consistency;  ///< Geometric consistency score
    float depth_coverage;         ///< Percentage of landmarks with valid depth

    // Timing
    double detection_time_ms;     ///< Detection processing time

    FacialLandmarks() : model(LandmarkModel::LANDMARKS_68), overall_confidence(0),
                       geometric_consistency(0), depth_coverage(0), detection_time_ms(0) {}

    bool isValid() const {
        return !points2D.empty() && overall_confidence > 0.5f;
    }

    size_t getPointCount() const {
        return points2D.size();
    }

    bool has3D() const {
        return !points3D.empty() && points3D.size() == points2D.size();
    }
};

/**
 * @brief Face detection bounding box with metadata
 */
struct FaceBoundingBox {
    cv::Rect2f bbox;              ///< Face bounding box in image coordinates
    float confidence;             ///< Detection confidence (0-1)
    float angle_yaw;              ///< Head rotation yaw angle (degrees)
    float angle_pitch;            ///< Head rotation pitch angle (degrees)
    float angle_roll;             ///< Head rotation roll angle (degrees)
    FaceQuality quality;          ///< Face quality assessment

    FaceBoundingBox() : confidence(0), angle_yaw(0), angle_pitch(0),
                       angle_roll(0), quality(FaceQuality::LOW) {}

    bool isValid() const {
        return confidence > 0.3f && bbox.area() > 0;
    }

    cv::Point2f getCenter() const {
        return cv::Point2f(bbox.x + bbox.width * 0.5f, bbox.y + bbox.height * 0.5f);
    }

    float getSize() const {
        return std::sqrt(bbox.width * bbox.height);
    }
};

/**
 * @brief Complete face detection result
 */
struct FaceDetectionResult {
    bool success;                              ///< Detection success flag
    std::vector<FaceBoundingBox> faces;        ///< Detected faces
    cv::Mat processed_image;                   ///< Pre-processed input image

    // Quality assessment
    float image_quality;          ///< Overall image quality (0-1)
    float lighting_quality;       ///< Lighting assessment (0-1)
    float focus_quality;          ///< Focus/sharpness assessment (0-1)

    // Performance metrics
    double detection_time_ms;     ///< Detection processing time
    double preprocessing_time_ms; ///< Pre-processing time

    // Error information
    FaceResultCode result_code;   ///< Specific result/error code
    std::string error_message;    ///< Human-readable error message

    FaceDetectionResult() : success(false), image_quality(0), lighting_quality(0),
                           focus_quality(0), detection_time_ms(0), preprocessing_time_ms(0),
                           result_code(FaceResultCode::ERROR_NO_FACE_DETECTED) {}

    bool hasSingleFace() const {
        return success && faces.size() == 1;
    }

    FaceBoundingBox getBestFace() const {
        if (faces.empty()) return FaceBoundingBox();

        // Return face with highest confidence
        auto best = std::max_element(faces.begin(), faces.end(),
            [](const FaceBoundingBox& a, const FaceBoundingBox& b) {
                return a.confidence < b.confidence;
            });
        return *best;
    }
};

/**
 * @brief 3D face mesh data
 */
struct Face3DMesh {
    std::vector<cv::Point3f> vertices;     ///< 3D mesh vertices (mm)
    std::vector<cv::Point3i> triangles;    ///< Triangle indices
    std::vector<cv::Point2f> texture_coords; ///< Texture coordinates
    cv::Mat texture_map;                   ///< Texture image

    // Quality metrics
    float mesh_completeness;    ///< Percentage of valid vertices (0-1)
    float surface_smoothness;   ///< Surface smoothness measure (0-1)
    float texture_quality;      ///< Texture map quality (0-1)

    Face3DMesh() : mesh_completeness(0), surface_smoothness(0), texture_quality(0) {}

    bool isValid() const {
        return !vertices.empty() && !triangles.empty() && mesh_completeness > 0.7f;
    }

    size_t getVertexCount() const { return vertices.size(); }
    size_t getTriangleCount() const { return triangles.size(); }
};

/**
 * @brief Complete 3D face model with geometric and photometric data
 */
struct Face3DModel {
    FacialLandmarks landmarks;        ///< 3D facial landmarks
    Face3DMesh mesh;                  ///< 3D face mesh
    cv::Mat depth_map;                ///< High-resolution depth map
    cv::Mat confidence_map;           ///< Depth confidence map

    // Geometric measurements
    float interpupillary_distance_mm; ///< IPD measurement
    float face_width_mm;              ///< Face width at cheekbones
    float face_height_mm;             ///< Face height (forehead to chin)
    float nose_bridge_height_mm;      ///< Nose bridge prominence

    // Quality assessment
    float reconstruction_quality;     ///< Overall reconstruction quality (0-1)
    float geometric_accuracy;         ///< Geometric accuracy estimate (0-1)
    float completeness_score;         ///< Model completeness (0-1)

    // Processing metadata
    double reconstruction_time_ms;    ///< Total reconstruction time
    std::string reconstruction_method; ///< Method used for reconstruction

    Face3DModel() : interpupillary_distance_mm(0), face_width_mm(0), face_height_mm(0),
                   nose_bridge_height_mm(0), reconstruction_quality(0),
                   geometric_accuracy(0), completeness_score(0), reconstruction_time_ms(0) {}

    bool isValid() const {
        return landmarks.isValid() && mesh.isValid() && reconstruction_quality > 0.8f;
    }

    bool isBankingGrade() const {
        return isValid() && reconstruction_quality > 0.95f &&
               geometric_accuracy > 0.95f && completeness_score > 0.9f;
    }
};

/**
 * @brief Banking-grade biometric template (encrypted)
 */
struct BiometricTemplate {
    // Template identification
    std::string template_id;          ///< Unique template identifier
    std::string subject_id;           ///< Subject identifier
    uint32_t template_version;        ///< Template format version

    // Encrypted biometric data
    std::vector<uint8_t> encrypted_features;    ///< AES-256 encrypted feature vector
    std::vector<uint8_t> encrypted_landmarks;   ///< Encrypted landmark data
    std::vector<uint8_t> encrypted_geometry;    ///< Encrypted geometric measurements

    // Cryptographic data
    std::array<uint8_t, 32> encryption_key_hash; ///< SHA-256 hash of encryption key
    std::array<uint8_t, 16> initialization_vector; ///< AES initialization vector
    std::array<uint8_t, 32> template_hash;      ///< Template integrity hash

    // Quality and compliance
    FaceQuality enrollment_quality;   ///< Quality at enrollment
    float template_quality_score;     ///< Template quality (0-1)
    bool iso_compliant;               ///< ISO/IEC 19794-5 compliance
    bool banking_certified;           ///< Banking grade certification

    // Metadata
    std::chrono::system_clock::time_point creation_time;
    std::chrono::system_clock::time_point last_updated;
    std::string creation_device_id;   ///< Device used for enrollment
    std::string encryption_algorithm; ///< Encryption method used

    // Template statistics
    size_t feature_count;             ///< Number of features in template
    size_t landmark_count;            ///< Number of landmarks stored
    float expected_far;               ///< Expected false accept rate
    float expected_frr;               ///< Expected false reject rate

    BiometricTemplate() : template_version(1), enrollment_quality(FaceQuality::LOW),
                         template_quality_score(0), iso_compliant(false),
                         banking_certified(false), feature_count(0), landmark_count(0),
                         expected_far(0), expected_frr(0) {}

    bool isValid() const {
        return !template_id.empty() && !encrypted_features.empty() &&
               template_quality_score > 0.8f;
    }

    bool isBankingGrade() const {
        return isValid() && banking_certified && expected_far < 0.001f && expected_frr < 0.03f;
    }

    size_t getSize() const {
        return encrypted_features.size() + encrypted_landmarks.size() + encrypted_geometry.size();
    }
};

/**
 * @brief Face matching result with detailed metrics
 */
struct FaceMatchResult {
    bool success;                     ///< Matching operation success
    float similarity_score;           ///< Similarity score (0-1)
    float confidence_level;           ///< Match confidence (0-1)
    bool is_match;                    ///< Boolean match decision

    // Banking-grade metrics
    float false_accept_rate;          ///< Estimated FAR for this match
    float false_reject_rate;          ///< Estimated FRR for this match
    float match_quality;              ///< Quality of matching process (0-1)

    // Performance metrics
    double matching_time_ms;          ///< Time taken for matching
    double template_load_time_ms;     ///< Time to load/decrypt template

    // Error information
    FaceResultCode result_code;       ///< Specific result/error code
    std::string error_message;        ///< Human-readable error message

    // Matching details
    std::string matched_template_id;  ///< ID of matched template (if any)
    std::vector<float> feature_similarities; ///< Per-feature similarity scores

    FaceMatchResult() : success(false), similarity_score(0), confidence_level(0),
                       is_match(false), false_accept_rate(1.0f), false_reject_rate(1.0f),
                       match_quality(0), matching_time_ms(0), template_load_time_ms(0),
                       result_code(FaceResultCode::ERROR_MATCHING_FAILED) {}

    bool isBankingGradeMatch() const {
        return success && is_match && confidence_level > 0.95f &&
               false_accept_rate < 0.001f && match_quality > 0.9f;
    }
};

/**
 * @brief Liveness detection result
 */
struct LivenessResult {
    bool is_live;                     ///< Overall liveness decision
    float liveness_score;             ///< Liveness confidence (0-1)
    LivenessMethod method_used;       ///< Detection method used

    // Individual test results
    bool depth_liveness_pass;         ///< Depth-based liveness test
    bool motion_liveness_pass;        ///< Motion-based liveness test
    bool texture_liveness_pass;       ///< Texture analysis liveness test

    // Detailed scores
    float depth_liveness_score;       ///< Depth liveness confidence
    float motion_liveness_score;      ///< Motion liveness confidence
    float texture_liveness_score;     ///< Texture liveness confidence

    // Attack detection
    bool presentation_attack_detected; ///< Presentation attack flag
    std::string attack_type;          ///< Type of attack detected (if any)
    float attack_confidence;          ///< Confidence in attack detection

    // Performance
    double processing_time_ms;        ///< Total processing time

    // Error information
    FaceResultCode result_code;       ///< Specific result/error code
    std::string error_message;        ///< Human-readable error message

    LivenessResult() : is_live(false), liveness_score(0),
                      method_used(LivenessMethod::PASSIVE_DEPTH),
                      depth_liveness_pass(false), motion_liveness_pass(false),
                      texture_liveness_pass(false), depth_liveness_score(0),
                      motion_liveness_score(0), texture_liveness_score(0),
                      presentation_attack_detected(true), attack_confidence(0),
                      processing_time_ms(0), result_code(FaceResultCode::ERROR_LIVENESS_CHECK_FAILED) {}

    bool isBankingGradeLive() const {
        return is_live && liveness_score > 0.95f && !presentation_attack_detected &&
               depth_liveness_pass && motion_liveness_pass;
    }
};

/**
 * @brief Face enrollment configuration
 */
struct EnrollmentConfig {
    // Quality requirements
    FaceQuality minimum_quality;      ///< Minimum acceptable face quality
    int required_samples;             ///< Number of face samples required
    float minimum_pose_variation;     ///< Required pose variation (degrees)

    // Capture settings
    float optimal_distance_mm;        ///< Optimal capture distance
    float distance_tolerance_mm;      ///< Distance tolerance
    cv::Size minimum_face_size;       ///< Minimum face size in pixels
    cv::Size maximum_face_size;       ///< Maximum face size in pixels

    // Liveness requirements
    bool require_liveness;            ///< Require liveness detection
    LivenessMethod liveness_method;   ///< Liveness detection method
    float liveness_threshold;         ///< Liveness acceptance threshold

    // Banking compliance
    bool banking_grade_required;      ///< Require banking grade quality
    bool iso_compliance_required;     ///< Require ISO compliance

    EnrollmentConfig() : minimum_quality(FaceQuality::MEDIUM), required_samples(3),
                        minimum_pose_variation(10.0f), optimal_distance_mm(450.0f),
                        distance_tolerance_mm(50.0f), minimum_face_size(200, 200),
                        maximum_face_size(800, 800), require_liveness(true),
                        liveness_method(LivenessMethod::MULTI_MODAL), liveness_threshold(0.8f),
                        banking_grade_required(false), iso_compliance_required(false) {}
};

/**
 * @brief Face enrollment progress information
 */
struct EnrollmentProgress {
    EnrollmentState current_state;    ///< Current enrollment state
    float completion_percentage;      ///< Overall completion (0-100)
    int samples_captured;             ///< Number of samples captured
    int samples_required;             ///< Total samples required

    // Current sample quality
    FaceQuality current_sample_quality; ///< Quality of current sample
    float pose_coverage;              ///< Pose variation coverage (0-1)
    bool liveness_passed;             ///< Current liveness status

    // Guidance for user
    std::string user_instruction;     ///< Instruction for user
    bool ready_to_capture;            ///< Ready to capture next sample

    // Error information
    FaceResultCode last_error;        ///< Last error encountered
    std::string error_message;        ///< Human-readable error

    EnrollmentProgress() : current_state(EnrollmentState::NOT_STARTED),
                          completion_percentage(0), samples_captured(0), samples_required(3),
                          current_sample_quality(FaceQuality::LOW), pose_coverage(0),
                          liveness_passed(false), ready_to_capture(false),
                          last_error(FaceResultCode::SUCCESS) {}
};

// Callback function types for asynchronous operations
using FaceDetectionCallback = std::function<void(const FaceDetectionResult&)>;
using LandmarkDetectionCallback = std::function<void(const FacialLandmarks&)>;
using Face3DReconstructionCallback = std::function<void(const Face3DModel&)>;
using TemplateGenerationCallback = std::function<void(const BiometricTemplate&)>;
using FaceMatchingCallback = std::function<void(const FaceMatchResult&)>;
using LivenessDetectionCallback = std::function<void(const LivenessResult&)>;
using EnrollmentProgressCallback = std::function<void(const EnrollmentProgress&)>;

// Smart pointer aliases for face types
template<typename T>
using UniqueFacePtr = std::unique_ptr<T>;

template<typename T>
using SharedFacePtr = std::shared_ptr<T>;

template<typename T>
using WeakFacePtr = std::weak_ptr<T>;

/// Helper function to convert FaceResultCode to string
std::string faceResultCodeToString(FaceResultCode code);

/// Current facial recognition API version
const core::Version FACE_API_VERSION{1, 0, 0, "banking"};

} // namespace face
} // namespace unlook