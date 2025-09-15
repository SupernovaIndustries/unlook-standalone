#include "unlook/face/FaceTypes.hpp"
#include <sstream>
#include <iomanip>

namespace unlook {
namespace face {

std::string faceResultCodeToString(FaceResultCode code) {
    switch (code) {
        case FaceResultCode::SUCCESS:
            return "Success";

        // Detection errors
        case FaceResultCode::ERROR_NO_FACE_DETECTED:
            return "No face detected in image";
        case FaceResultCode::ERROR_MULTIPLE_FACES_DETECTED:
            return "Multiple faces detected, expected single face";
        case FaceResultCode::ERROR_FACE_TOO_SMALL:
            return "Face too small for reliable processing";
        case FaceResultCode::ERROR_FACE_TOO_LARGE:
            return "Face too large, exceeds processing limits";
        case FaceResultCode::ERROR_FACE_OUT_OF_BOUNDS:
            return "Face extends beyond image boundaries";
        case FaceResultCode::ERROR_POOR_FACE_QUALITY:
            return "Face quality insufficient for processing";

        // Landmark errors
        case FaceResultCode::ERROR_LANDMARK_DETECTION_FAILED:
            return "Facial landmark detection failed";
        case FaceResultCode::ERROR_INSUFFICIENT_LANDMARKS:
            return "Insufficient number of landmarks detected";
        case FaceResultCode::ERROR_LANDMARK_QUALITY_LOW:
            return "Landmark detection quality too low";

        // 3D reconstruction errors
        case FaceResultCode::ERROR_DEPTH_DATA_INVALID:
            return "Depth data invalid or corrupted";
        case FaceResultCode::ERROR_STEREO_RECONSTRUCTION_FAILED:
            return "Stereo 3D reconstruction failed";
        case FaceResultCode::ERROR_3D_MODEL_INCOMPLETE:
            return "3D face model incomplete or invalid";
        case FaceResultCode::ERROR_MESH_GENERATION_FAILED:
            return "3D mesh generation failed";

        // Template errors
        case FaceResultCode::ERROR_TEMPLATE_GENERATION_FAILED:
            return "Biometric template generation failed";
        case FaceResultCode::ERROR_TEMPLATE_INVALID:
            return "Biometric template invalid or corrupted";
        case FaceResultCode::ERROR_TEMPLATE_CORRUPTED:
            return "Biometric template corrupted";
        case FaceResultCode::ERROR_TEMPLATE_VERSION_MISMATCH:
            return "Template version mismatch";

        // Matching errors
        case FaceResultCode::ERROR_MATCHING_FAILED:
            return "Face matching operation failed";
        case FaceResultCode::ERROR_TEMPLATE_DATABASE_ERROR:
            return "Template database error";
        case FaceResultCode::ERROR_SIMILARITY_COMPUTATION_FAILED:
            return "Similarity computation failed";

        // Liveness errors
        case FaceResultCode::ERROR_LIVENESS_CHECK_FAILED:
            return "Liveness detection failed";
        case FaceResultCode::ERROR_PRESENTATION_ATTACK_DETECTED:
            return "Presentation attack detected";
        case FaceResultCode::ERROR_DEPTH_LIVENESS_FAILED:
            return "Depth-based liveness check failed";
        case FaceResultCode::ERROR_MOTION_LIVENESS_FAILED:
            return "Motion-based liveness check failed";

        // Security errors
        case FaceResultCode::ERROR_ENCRYPTION_FAILED:
            return "Template encryption failed";
        case FaceResultCode::ERROR_DECRYPTION_FAILED:
            return "Template decryption failed";
        case FaceResultCode::ERROR_AUTHENTICATION_FAILED:
            return "Authentication failed";
        case FaceResultCode::ERROR_AUTHORIZATION_DENIED:
            return "Authorization denied";

        // Hardware/Integration errors
        case FaceResultCode::ERROR_CAMERA_NOT_AVAILABLE:
            return "Camera not available";
        case FaceResultCode::ERROR_DEPTH_SENSOR_FAILED:
            return "Depth sensor failed";
        case FaceResultCode::ERROR_LED_CONTROL_FAILED:
            return "LED control failed";
        case FaceResultCode::ERROR_SUPERNOVA_ML_ERROR:
            return "Supernova ML service error";

        default:
            return "Unknown face error code: " + std::to_string(static_cast<int32_t>(code));
    }
}

// FaceDetectionConfig implementations
bool FaceDetectionConfig::validate() const {
    if (scale_factor <= 1.0f) return false;
    if (min_neighbors < 1) return false;
    if (min_face_size.width <= 0 || min_face_size.height <= 0) return false;
    if (max_face_size.width <= min_face_size.width || max_face_size.height <= min_face_size.height) return false;
    if (confidence_threshold < 0.0f || confidence_threshold > 1.0f) return false;
    if (max_yaw_angle < 0.0f || max_yaw_angle > 90.0f) return false;
    if (max_pitch_angle < 0.0f || max_pitch_angle > 90.0f) return false;
    if (max_roll_angle < 0.0f || max_roll_angle > 90.0f) return false;
    if (banking_confidence_threshold < 0.0f || banking_confidence_threshold > 1.0f) return false;
    return true;
}

std::string FaceDetectionConfig::toString() const {
    std::stringstream ss;
    ss << std::fixed << std::setprecision(2);
    ss << "FaceDetectionConfig{";
    ss << "scale_factor=" << scale_factor;
    ss << ", min_neighbors=" << min_neighbors;
    ss << ", min_face_size=" << min_face_size.width << "x" << min_face_size.height;
    ss << ", max_face_size=" << max_face_size.width << "x" << max_face_size.height;
    ss << ", confidence_threshold=" << confidence_threshold;
    ss << ", banking_mode=" << (banking_mode ? "true" : "false");
    ss << "}";
    return ss.str();
}

// LandmarkExtractionConfig implementations
bool LandmarkExtractionConfig::validate() const {
    if (confidence_threshold < 0.0f || confidence_threshold > 1.0f) return false;
    if (min_landmark_visibility < 0.0f || min_landmark_visibility > 1.0f) return false;
    if (max_landmark_deviation < 0.0f) return false;
    if (depth_weight < 0.0f || depth_weight > 1.0f) return false;
    if (stereo_confidence_threshold < 0.0f || stereo_confidence_threshold > 1.0f) return false;
    if (temporal_alpha < 0.0f || temporal_alpha > 1.0f) return false;
    if (banking_confidence_threshold < 0.0f || banking_confidence_threshold > 1.0f) return false;
    return true;
}

std::string LandmarkExtractionConfig::toString() const {
    std::stringstream ss;
    ss << std::fixed << std::setprecision(2);
    ss << "LandmarkExtractionConfig{";
    ss << "model_type=" << static_cast<int>(model_type);
    ss << ", confidence_threshold=" << confidence_threshold;
    ss << ", enable_3d=" << (enable_3d_estimation ? "true" : "false");
    ss << ", banking_mode=" << (banking_mode ? "true" : "false");
    ss << "}";
    return ss.str();
}

// Face3DReconstructionConfig implementations
bool Face3DReconstructionConfig::validate() const {
    if (min_depth_mm <= 0.0f || max_depth_mm <= min_depth_mm) return false;
    if (depth_confidence_threshold < 0.0f || depth_confidence_threshold > 1.0f) return false;
    if (voxel_size_mm <= 0.0f) return false;
    if (max_vertices <= 0 || max_triangles <= 0) return false;
    if (landmark_weight < 0.0f || landmark_weight > 1.0f) return false;
    if (regularization_weight < 0.0f) return false;
    if (max_iterations <= 0) return false;
    if (min_mesh_completeness < 0.0f || min_mesh_completeness > 1.0f) return false;
    if (banking_quality_threshold < 0.0f || banking_quality_threshold > 1.0f) return false;
    return true;
}

std::string Face3DReconstructionConfig::toString() const {
    std::stringstream ss;
    ss << std::fixed << std::setprecision(2);
    ss << "Face3DReconstructionConfig{";
    ss << "method=" << static_cast<int>(method);
    ss << ", min_depth=" << min_depth_mm << "mm";
    ss << ", max_depth=" << max_depth_mm << "mm";
    ss << ", voxel_size=" << voxel_size_mm << "mm";
    ss << ", banking_mode=" << (banking_mode ? "true" : "false");
    ss << "}";
    return ss.str();
}

// FaceEncodingConfig implementations
bool FaceEncodingConfig::validate() const {
    if (feature_vector_size <= 0) return false;
    if (pca_components > feature_vector_size) return false;
    if (min_feature_quality < 0.0f || min_feature_quality > 1.0f) return false;
    if (augmentation_samples < 1) return false;
    if (banking_quality_threshold < 0.0f || banking_quality_threshold > 1.0f) return false;
    return true;
}

std::string FaceEncodingConfig::toString() const {
    std::stringstream ss;
    ss << std::fixed << std::setprecision(2);
    ss << "FaceEncodingConfig{";
    ss << "method=" << static_cast<int>(method);
    ss << ", feature_size=" << feature_vector_size;
    ss << ", enable_encryption=" << (enable_encryption ? "true" : "false");
    ss << ", banking_mode=" << (banking_certification_mode ? "true" : "false");
    ss << "}";
    return ss.str();
}

// FaceMatchingConfig implementations
bool FaceMatchingConfig::validate() const {
    if (verification_threshold < 0.0f || verification_threshold > 1.0f) return false;
    if (identification_threshold < 0.0f || identification_threshold > 1.0f) return false;
    if (banking_threshold < 0.0f || banking_threshold > 1.0f) return false;
    if (min_template_quality < 0.0f || min_template_quality > 1.0f) return false;
    if (min_probe_quality < 0.0f || min_probe_quality > 1.0f) return false;
    if (target_far < 0.0f || target_far > 1.0f) return false;
    if (target_frr < 0.0f || target_frr > 1.0f) return false;
    if (cache_size_mb <= 0) return false;
    if (max_database_size <= 0) return false;
    return true;
}

std::string FaceMatchingConfig::toString() const {
    std::stringstream ss;
    ss << std::fixed << std::setprecision(4);
    ss << "FaceMatchingConfig{";
    ss << "method=" << static_cast<int>(method);
    ss << ", verification_threshold=" << verification_threshold;
    ss << ", target_far=" << target_far;
    ss << ", target_frr=" << target_frr;
    ss << ", banking_mode=" << (banking_audit_mode ? "true" : "false");
    ss << "}";
    return ss.str();
}

// LivenessDetectionConfig implementations
bool LivenessDetectionConfig::validate() const {
    if (min_depth_variation_mm < 0.0f) return false;
    if (max_depth_noise_mm < 0.0f) return false;
    if (depth_confidence_threshold < 0.0f || depth_confidence_threshold > 1.0f) return false;
    if (min_motion_magnitude < 0.0f) return false;
    if (max_motion_magnitude < min_motion_magnitude) return false;
    if (motion_analysis_frames <= 0) return false;
    if (texture_quality_threshold < 0.0f || texture_quality_threshold > 1.0f) return false;
    if (temporal_window_frames <= 0) return false;
    if (banking_confidence_threshold < 0.0f || banking_confidence_threshold > 1.0f) return false;
    return true;
}

std::string LivenessDetectionConfig::toString() const {
    std::stringstream ss;
    ss << std::fixed << std::setprecision(2);
    ss << "LivenessDetectionConfig{";
    ss << "enable_depth=" << (enable_depth_liveness ? "true" : "false");
    ss << ", enable_motion=" << (enable_motion_liveness ? "true" : "false");
    ss << ", enable_texture=" << (enable_texture_liveness ? "true" : "false");
    ss << ", banking_mode=" << (banking_grade_mode ? "true" : "false");
    ss << "}";
    return ss.str();
}

// SupernovaMLConfig implementations
bool SupernovaMLConfig::validate() const {
    if (server_endpoint.empty()) return false;
    if (api_version.empty()) return false;
    if (client_id.empty()) return false;
    if (timeout_seconds <= 0) return false;
    if (max_retries < 0) return false;
    if (cache_ttl_seconds <= 0) return false;
    if (max_batch_size <= 0) return false;
    return true;
}

std::string SupernovaMLConfig::toString() const {
    std::stringstream ss;
    ss << "SupernovaMLConfig{";
    ss << "endpoint=" << server_endpoint;
    ss << ", version=" << api_version;
    ss << ", banking_mode=" << (banking_mode ? "true" : "false");
    ss << ", compliance=" << compliance_level;
    ss << "}";
    return ss.str();
}

// FaceRecognitionSystemConfig implementations
bool FaceRecognitionSystemConfig::validate() const {
    if (!detector_config.validate()) return false;
    if (!landmark_config.validate()) return false;
    if (!reconstruction_config.validate()) return false;
    if (!encoding_config.validate()) return false;
    if (!matching_config.validate()) return false;
    if (!liveness_config.validate()) return false;
    if (!supernova_config.validate()) return false;
    if (target_far < 0.0f || target_far > 1.0f) return false;
    if (target_frr < 0.0f || target_frr > 1.0f) return false;
    if (max_processing_threads <= 0) return false;
    return true;
}

std::string FaceRecognitionSystemConfig::toString() const {
    std::stringstream ss;
    ss << "FaceRecognitionSystemConfig{";
    ss << "banking_mode=" << (banking_grade_mode ? "true" : "false");
    ss << ", target_far=" << target_far;
    ss << ", target_frr=" << target_frr;
    ss << ", threads=" << max_processing_threads;
    ss << "}";
    return ss.str();
}

// FaceRecognitionMetrics implementations
std::string FaceRecognitionMetrics::toString() const {
    std::stringstream ss;
    ss << std::fixed << std::setprecision(2);
    ss << "FaceRecognitionMetrics{";
    ss << "detection_time=" << avg_detection_time_ms << "ms";
    ss << ", recognition_time=" << avg_recognition_time_ms << "ms";
    ss << ", accuracy=" << (recognition_accuracy * 100.0f) << "%";
    ss << ", liveness_rate=" << (liveness_success_rate * 100.0f) << "%";
    ss << ", uptime=" << system_uptime_hours << "h";
    ss << ", memory=" << memory_usage_mb << "MB";
    ss << ", cpu=" << cpu_usage_percent << "%";
    ss << "}";
    return ss.str();
}

} // namespace face
} // namespace unlook