#include "unlook/face/FaceTypes.hpp"
#include "unlook/face/FaceException.hpp"
#include "unlook/core/logging.hpp"
#include <opencv2/core.hpp>
#include <algorithm>
#include <numeric>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <sstream>

namespace unlook {
namespace face {

/**
 * @brief Banking compliance and ISO/IEC 19794-5 validation system
 *
 * This class provides comprehensive validation for banking-grade facial
 * recognition systems according to international standards and banking
 * regulatory requirements.
 */
class BankingComplianceValidator {
public:
    /**
     * @brief ISO/IEC 19794-5 compliance requirements
     */
    struct ISO19794_5_Requirements {
        // Image quality requirements
        float min_image_resolution = 90.0f;        // Minimum 90 pixels between eye centers
        float min_face_width_pixels = 120.0f;      // Minimum face width in pixels
        float min_face_height_pixels = 150.0f;     // Minimum face height in pixels
        float max_head_rotation_degrees = 15.0f;   // Maximum head rotation
        float min_eye_distance_pixels = 90.0f;     // Minimum inter-ocular distance

        // Quality metrics
        float min_sharpness_score = 0.7f;          // Minimum image sharpness
        float min_illumination_uniformity = 0.6f;  // Minimum illumination uniformity
        float max_compression_artifacts = 0.3f;    // Maximum compression artifacts
        float min_contrast_score = 0.5f;           // Minimum contrast

        // Pose and expression requirements
        float max_yaw_angle = 15.0f;               // Maximum yaw rotation
        float max_pitch_angle = 15.0f;             // Maximum pitch rotation
        float max_roll_angle = 15.0f;              // Maximum roll rotation
        bool require_neutral_expression = true;    // Require neutral facial expression

        // Color and format requirements
        bool require_color_image = false;          // Color vs grayscale
        int min_bit_depth = 8;                     // Minimum bit depth
        float min_color_fidelity = 0.8f;           // Color reproduction accuracy

        bool validate() const {
            return min_image_resolution > 0 && min_face_width_pixels > 0 &&
                   min_face_height_pixels > 0 && max_head_rotation_degrees > 0 &&
                   min_sharpness_score >= 0 && min_sharpness_score <= 1.0f;
        }
    };

    /**
     * @brief Banking regulatory requirements
     */
    struct BankingRequirements {
        // Performance requirements
        float max_false_accept_rate = 0.001f;      // Maximum FAR (0.1%)
        float max_false_reject_rate = 0.03f;       // Maximum FRR (3%)
        float min_template_quality = 0.95f;        // Minimum template quality
        float min_liveness_confidence = 0.95f;     // Minimum liveness detection confidence

        // Security requirements
        bool require_template_encryption = true;   // Mandatory template encryption
        bool require_audit_trail = true;           // Mandatory audit logging
        bool require_velocity_checking = true;     // Mandatory velocity limits
        bool require_presentation_attack_detection = true; // PAD required

        // Data protection requirements
        bool require_gdpr_compliance = true;       // GDPR compliance
        bool require_data_minimization = true;     // Data minimization
        int max_template_retention_days = 2555;    // Maximum retention (7 years)
        bool require_user_consent = true;          // Explicit user consent

        // Operational requirements
        float max_processing_time_ms = 500.0f;     // Maximum processing time
        float min_availability_percentage = 99.9f; // Minimum system availability
        bool require_fail_safe_mode = true;        // Fail-safe operation required
        bool require_operator_override = true;     // Manual override capability

        bool validate() const {
            return max_false_accept_rate > 0 && max_false_accept_rate < 1.0f &&
                   max_false_reject_rate > 0 && max_false_reject_rate < 1.0f &&
                   min_template_quality >= 0.9f && min_liveness_confidence >= 0.9f;
        }
    };

    /**
     * @brief Compliance validation result
     */
    struct ComplianceResult {
        bool iso_19794_5_compliant = false;
        bool banking_compliant = false;
        bool overall_compliant = false;

        std::vector<std::string> iso_violations;
        std::vector<std::string> banking_violations;
        std::vector<std::string> recommendations;

        float compliance_score = 0.0f;            // Overall compliance score (0-1)
        float iso_compliance_score = 0.0f;        // ISO compliance score
        float banking_compliance_score = 0.0f;    // Banking compliance score

        std::chrono::system_clock::time_point validation_timestamp;
        std::string validator_version = "1.0.0";
        std::string validation_id;

        std::map<std::string, float> detailed_metrics;
    };

private:
    ISO19794_5_Requirements iso_requirements_;
    BankingRequirements banking_requirements_;
    std::string last_error_;

public:
    BankingComplianceValidator(const ISO19794_5_Requirements& iso_req = ISO19794_5_Requirements{},
                              const BankingRequirements& banking_req = BankingRequirements{})
        : iso_requirements_(iso_req), banking_requirements_(banking_req) {

        if (!iso_requirements_.validate() || !banking_requirements_.validate()) {
            UNLOOK_LOG_WARN("Invalid compliance requirements configuration");
        }
    }

    /**
     * @brief Validate face image compliance with ISO/IEC 19794-5
     */
    FaceResultCode validateISO19794_5_Image(const cv::Mat& face_image,
                                            const FacialLandmarks& landmarks,
                                            ComplianceResult& result) {
        try {
            result.iso_violations.clear();
            float iso_score = 1.0f;

            // Image dimension validation
            float face_width = computeFaceWidth(landmarks);
            float face_height = computeFaceHeight(landmarks);

            if (face_width < iso_requirements_.min_face_width_pixels) {
                result.iso_violations.push_back("Face width below minimum requirement: " +
                                               std::to_string(face_width) + " < " +
                                               std::to_string(iso_requirements_.min_face_width_pixels));
                iso_score *= 0.8f;
            }

            if (face_height < iso_requirements_.min_face_height_pixels) {
                result.iso_violations.push_back("Face height below minimum requirement: " +
                                               std::to_string(face_height) + " < " +
                                               std::to_string(iso_requirements_.min_face_height_pixels));
                iso_score *= 0.8f;
            }

            // Inter-ocular distance validation
            float eye_distance = computeInterOcularDistance(landmarks);
            if (eye_distance < iso_requirements_.min_eye_distance_pixels) {
                result.iso_violations.push_back("Inter-ocular distance below minimum: " +
                                               std::to_string(eye_distance) + " < " +
                                               std::to_string(iso_requirements_.min_eye_distance_pixels));
                iso_score *= 0.7f;
            }

            // Image quality validation
            float sharpness = computeImageSharpness(face_image);
            if (sharpness < iso_requirements_.min_sharpness_score) {
                result.iso_violations.push_back("Image sharpness below minimum: " +
                                               std::to_string(sharpness) + " < " +
                                               std::to_string(iso_requirements_.min_sharpness_score));
                iso_score *= 0.6f;
            }

            float illumination_uniformity = computeIlluminationUniformity(face_image);
            if (illumination_uniformity < iso_requirements_.min_illumination_uniformity) {
                result.iso_violations.push_back("Illumination uniformity below minimum: " +
                                               std::to_string(illumination_uniformity) + " < " +
                                               std::to_string(iso_requirements_.min_illumination_uniformity));
                iso_score *= 0.7f;
            }

            float contrast = computeImageContrast(face_image);
            if (contrast < iso_requirements_.min_contrast_score) {
                result.iso_violations.push_back("Image contrast below minimum: " +
                                               std::to_string(contrast) + " < " +
                                               std::to_string(iso_requirements_.min_contrast_score));
                iso_score *= 0.8f;
            }

            // Pose validation
            cv::Vec3f pose_angles = estimateHeadPose(landmarks);

            if (std::abs(pose_angles[0]) > iso_requirements_.max_yaw_angle) {
                result.iso_violations.push_back("Yaw angle exceeds maximum: " +
                                               std::to_string(std::abs(pose_angles[0])) + " > " +
                                               std::to_string(iso_requirements_.max_yaw_angle));
                iso_score *= 0.5f;
            }

            if (std::abs(pose_angles[1]) > iso_requirements_.max_pitch_angle) {
                result.iso_violations.push_back("Pitch angle exceeds maximum: " +
                                               std::to_string(std::abs(pose_angles[1])) + " > " +
                                               std::to_string(iso_requirements_.max_pitch_angle));
                iso_score *= 0.5f;
            }

            if (std::abs(pose_angles[2]) > iso_requirements_.max_roll_angle) {
                result.iso_violations.push_back("Roll angle exceeds maximum: " +
                                               std::to_string(std::abs(pose_angles[2])) + " > " +
                                               std::to_string(iso_requirements_.max_roll_angle));
                iso_score *= 0.5f;
            }

            // Color and format validation
            if (iso_requirements_.require_color_image && face_image.channels() != 3) {
                result.iso_violations.push_back("Color image required but grayscale provided");
                iso_score *= 0.9f;
            }

            int bit_depth = (face_image.depth() == CV_8U) ? 8 :
                           (face_image.depth() == CV_16U) ? 16 : 32;
            if (bit_depth < iso_requirements_.min_bit_depth) {
                result.iso_violations.push_back("Bit depth below minimum: " +
                                               std::to_string(bit_depth) + " < " +
                                               std::to_string(iso_requirements_.min_bit_depth));
                iso_score *= 0.9f;
            }

            // Expression validation
            if (iso_requirements_.require_neutral_expression) {
                float expression_neutrality = assessExpressionNeutrality(landmarks);
                if (expression_neutrality < 0.8f) {
                    result.iso_violations.push_back("Non-neutral facial expression detected: " +
                                                   std::to_string(expression_neutrality));
                    iso_score *= 0.6f;
                }
            }

            result.iso_compliance_score = std::max(0.0f, iso_score);
            result.iso_19794_5_compliant = (result.iso_violations.empty() && iso_score >= 0.9f);

            // Store detailed metrics
            result.detailed_metrics["face_width_pixels"] = face_width;
            result.detailed_metrics["face_height_pixels"] = face_height;
            result.detailed_metrics["eye_distance_pixels"] = eye_distance;
            result.detailed_metrics["image_sharpness"] = sharpness;
            result.detailed_metrics["illumination_uniformity"] = illumination_uniformity;
            result.detailed_metrics["image_contrast"] = contrast;
            result.detailed_metrics["yaw_angle_degrees"] = pose_angles[0];
            result.detailed_metrics["pitch_angle_degrees"] = pose_angles[1];
            result.detailed_metrics["roll_angle_degrees"] = pose_angles[2];

            return FaceResultCode::SUCCESS;

        } catch (const std::exception& e) {
            last_error_ = "Exception during ISO validation: " + std::string(e.what());
            return FaceResultCode::ERROR_TEMPLATE_INVALID;
        }
    }

    /**
     * @brief Validate biometric template compliance with banking requirements
     */
    FaceResultCode validateBankingTemplate(const BiometricTemplate& template_data,
                                          const FaceMatchResult& performance_data,
                                          ComplianceResult& result) {
        try {
            result.banking_violations.clear();
            float banking_score = 1.0f;

            // Performance requirements
            if (performance_data.false_accept_rate > banking_requirements_.max_false_accept_rate) {
                result.banking_violations.push_back("False Accept Rate exceeds maximum: " +
                                                   std::to_string(performance_data.false_accept_rate) + " > " +
                                                   std::to_string(banking_requirements_.max_false_accept_rate));
                banking_score *= 0.3f; // Critical violation
            }

            if (performance_data.false_reject_rate > banking_requirements_.max_false_reject_rate) {
                result.banking_violations.push_back("False Reject Rate exceeds maximum: " +
                                                   std::to_string(performance_data.false_reject_rate) + " > " +
                                                   std::to_string(banking_requirements_.max_false_reject_rate));
                banking_score *= 0.5f;
            }

            if (template_data.template_quality_score < banking_requirements_.min_template_quality) {
                result.banking_violations.push_back("Template quality below minimum: " +
                                                   std::to_string(template_data.template_quality_score) + " < " +
                                                   std::to_string(banking_requirements_.min_template_quality));
                banking_score *= 0.6f;
            }

            // Security requirements
            if (banking_requirements_.require_template_encryption && template_data.encrypted_features.empty()) {
                result.banking_violations.push_back("Template encryption required but not present");
                banking_score *= 0.4f; // Critical security violation
            }

            if (banking_requirements_.require_template_encryption) {
                // Validate encryption integrity
                if (!validateTemplateEncryption(template_data)) {
                    result.banking_violations.push_back("Template encryption validation failed");
                    banking_score *= 0.3f;
                }
            }

            // Banking certification validation
            if (!template_data.banking_certified) {
                result.banking_violations.push_back("Template not banking-certified");
                banking_score *= 0.7f;
            }

            if (!template_data.iso_compliant) {
                result.banking_violations.push_back("Template not ISO compliant");
                banking_score *= 0.8f;
            }

            // Template integrity validation
            if (!validateTemplateIntegrity(template_data)) {
                result.banking_violations.push_back("Template integrity validation failed");
                banking_score *= 0.2f; // Critical violation
            }

            // Data protection compliance
            if (banking_requirements_.require_gdpr_compliance) {
                if (!validateGDPRCompliance(template_data)) {
                    result.banking_violations.push_back("GDPR compliance validation failed");
                    banking_score *= 0.8f;
                }
            }

            // Processing time validation
            if (performance_data.matching_time_ms > banking_requirements_.max_processing_time_ms) {
                result.banking_violations.push_back("Processing time exceeds maximum: " +
                                                   std::to_string(performance_data.matching_time_ms) + "ms > " +
                                                   std::to_string(banking_requirements_.max_processing_time_ms) + "ms");
                banking_score *= 0.9f;
            }

            result.banking_compliance_score = std::max(0.0f, banking_score);
            result.banking_compliant = (result.banking_violations.empty() && banking_score >= 0.95f);

            // Store banking-specific metrics
            result.detailed_metrics["template_quality"] = template_data.template_quality_score;
            result.detailed_metrics["false_accept_rate"] = performance_data.false_accept_rate;
            result.detailed_metrics["false_reject_rate"] = performance_data.false_reject_rate;
            result.detailed_metrics["processing_time_ms"] = performance_data.matching_time_ms;
            result.detailed_metrics["encryption_validated"] = validateTemplateEncryption(template_data) ? 1.0f : 0.0f;

            return FaceResultCode::SUCCESS;

        } catch (const std::exception& e) {
            last_error_ = "Exception during banking validation: " + std::string(e.what());
            return FaceResultCode::ERROR_TEMPLATE_INVALID;
        }
    }

    /**
     * @brief Perform comprehensive compliance validation
     */
    FaceResultCode performComprehensiveValidation(const cv::Mat& face_image,
                                                 const FacialLandmarks& landmarks,
                                                 const BiometricTemplate& template_data,
                                                 const FaceMatchResult& performance_data,
                                                 const LivenessResult& liveness_result,
                                                 ComplianceResult& result) {
        try {
            result = ComplianceResult{};
            result.validation_timestamp = std::chrono::system_clock::now();
            result.validation_id = generateValidationId();

            // ISO/IEC 19794-5 validation
            FaceResultCode iso_result = validateISO19794_5_Image(face_image, landmarks, result);
            if (iso_result != FaceResultCode::SUCCESS) {
                return iso_result;
            }

            // Banking requirements validation
            FaceResultCode banking_result = validateBankingTemplate(template_data, performance_data, result);
            if (banking_result != FaceResultCode::SUCCESS) {
                return banking_result;
            }

            // Liveness detection validation
            if (banking_requirements_.require_presentation_attack_detection) {
                if (liveness_result.liveness_score < banking_requirements_.min_liveness_confidence) {
                    result.banking_violations.push_back("Liveness confidence below minimum: " +
                                                       std::to_string(liveness_result.liveness_score) + " < " +
                                                       std::to_string(banking_requirements_.min_liveness_confidence));
                    result.banking_compliance_score *= 0.7f;
                }

                if (liveness_result.presentation_attack_detected) {
                    result.banking_violations.push_back("Presentation attack detected");
                    result.banking_compliance_score *= 0.1f; // Critical violation
                }
            }

            // Overall compliance assessment
            result.compliance_score = (result.iso_compliance_score * 0.4f +
                                     result.banking_compliance_score * 0.6f);

            result.overall_compliant = result.iso_19794_5_compliant &&
                                     result.banking_compliant &&
                                     result.compliance_score >= 0.95f;

            // Generate recommendations
            generateComplianceRecommendations(result);

            UNLOOK_LOG_INFO("Compliance validation completed: Overall score {:.3f}, ISO: {}, Banking: {}",
                           result.compliance_score,
                           result.iso_19794_5_compliant ? "PASS" : "FAIL",
                           result.banking_compliant ? "PASS" : "FAIL");

            return FaceResultCode::SUCCESS;

        } catch (const std::exception& e) {
            last_error_ = "Exception during comprehensive validation: " + std::string(e.what());
            return FaceResultCode::ERROR_TEMPLATE_INVALID;
        }
    }

    /**
     * @brief Generate compliance report
     */
    std::string generateComplianceReport(const ComplianceResult& result) {
        std::stringstream report;

        report << "=== BANKING COMPLIANCE VALIDATION REPORT ===\n";
        report << "Validation ID: " << result.validation_id << "\n";
        report << "Timestamp: " << formatTimestamp(result.validation_timestamp) << "\n";
        report << "Validator Version: " << result.validator_version << "\n\n";

        report << "OVERALL COMPLIANCE: " << (result.overall_compliant ? "PASS" : "FAIL") << "\n";
        report << "Compliance Score: " << std::fixed << std::setprecision(3) << result.compliance_score << "\n\n";

        report << "=== ISO/IEC 19794-5 COMPLIANCE ===\n";
        report << "Status: " << (result.iso_19794_5_compliant ? "COMPLIANT" : "NON-COMPLIANT") << "\n";
        report << "Score: " << std::fixed << std::setprecision(3) << result.iso_compliance_score << "\n";

        if (!result.iso_violations.empty()) {
            report << "Violations:\n";
            for (size_t i = 0; i < result.iso_violations.size(); ++i) {
                report << "  " << (i + 1) << ". " << result.iso_violations[i] << "\n";
            }
        }
        report << "\n";

        report << "=== BANKING REQUIREMENTS COMPLIANCE ===\n";
        report << "Status: " << (result.banking_compliant ? "COMPLIANT" : "NON-COMPLIANT") << "\n";
        report << "Score: " << std::fixed << std::setprecision(3) << result.banking_compliance_score << "\n";

        if (!result.banking_violations.empty()) {
            report << "Violations:\n";
            for (size_t i = 0; i < result.banking_violations.size(); ++i) {
                report << "  " << (i + 1) << ". " << result.banking_violations[i] << "\n";
            }
        }
        report << "\n";

        report << "=== DETAILED METRICS ===\n";
        for (const auto& metric : result.detailed_metrics) {
            report << metric.first << ": " << std::fixed << std::setprecision(3) << metric.second << "\n";
        }
        report << "\n";

        if (!result.recommendations.empty()) {
            report << "=== RECOMMENDATIONS ===\n";
            for (size_t i = 0; i < result.recommendations.size(); ++i) {
                report << "  " << (i + 1) << ". " << result.recommendations[i] << "\n";
            }
        }

        report << "\n=== END OF REPORT ===\n";

        return report.str();
    }

    /**
     * @brief Save compliance report to file
     */
    FaceResultCode saveComplianceReport(const ComplianceResult& result,
                                       const std::string& filename) {
        try {
            std::ofstream file(filename);
            if (!file.is_open()) {
                last_error_ = "Cannot open file for writing: " + filename;
                return FaceResultCode::ERROR_TEMPLATE_INVALID;
            }

            file << generateComplianceReport(result);
            file.close();

            return FaceResultCode::SUCCESS;

        } catch (const std::exception& e) {
            last_error_ = "Exception saving compliance report: " + std::string(e.what());
            return FaceResultCode::ERROR_TEMPLATE_INVALID;
        }
    }

    /**
     * @brief Get last error message
     */
    std::string getLastError() const {
        return last_error_;
    }

    /**
     * @brief Set ISO requirements
     */
    void setISORequirements(const ISO19794_5_Requirements& requirements) {
        iso_requirements_ = requirements;
    }

    /**
     * @brief Set banking requirements
     */
    void setBankingRequirements(const BankingRequirements& requirements) {
        banking_requirements_ = requirements;
    }

private:
    /**
     * @brief Compute face width from landmarks
     */
    float computeFaceWidth(const FacialLandmarks& landmarks) {
        if (landmarks.points2D.size() < 17) return 0.0f;

        const auto& left_jaw = landmarks.points2D[0];   // Left jaw point
        const auto& right_jaw = landmarks.points2D[16]; // Right jaw point

        return std::sqrt((left_jaw.x - right_jaw.x) * (left_jaw.x - right_jaw.x) +
                        (left_jaw.y - right_jaw.y) * (left_jaw.y - right_jaw.y));
    }

    /**
     * @brief Compute face height from landmarks
     */
    float computeFaceHeight(const FacialLandmarks& landmarks) {
        if (landmarks.points2D.size() < 28) return 0.0f;

        const auto& forehead = landmarks.points2D[27]; // Nose bridge (approximate forehead)
        const auto& chin = landmarks.points2D[8];      // Chin center

        return std::sqrt((forehead.x - chin.x) * (forehead.x - chin.x) +
                        (forehead.y - chin.y) * (forehead.y - chin.y));
    }

    /**
     * @brief Compute inter-ocular distance
     */
    float computeInterOcularDistance(const FacialLandmarks& landmarks) {
        if (landmarks.points2D.size() < 46) return 0.0f;

        const auto& left_eye = landmarks.points2D[36];  // Left eye outer corner
        const auto& right_eye = landmarks.points2D[45]; // Right eye outer corner

        return std::sqrt((left_eye.x - right_eye.x) * (left_eye.x - right_eye.x) +
                        (left_eye.y - right_eye.y) * (left_eye.y - right_eye.y));
    }

    /**
     * @brief Compute image sharpness using Laplacian variance
     */
    float computeImageSharpness(const cv::Mat& image) {
        cv::Mat gray, laplacian;

        if (image.channels() == 3) {
            cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
        } else {
            gray = image.clone();
        }

        cv::Laplacian(gray, laplacian, CV_64F);
        cv::Scalar mean, stddev;
        cv::meanStdDev(laplacian, mean, stddev);

        // Normalize variance to 0-1 range
        double variance = stddev[0] * stddev[0];
        return std::min(1.0f, static_cast<float>(variance / 1000.0));
    }

    /**
     * @brief Compute illumination uniformity
     */
    float computeIlluminationUniformity(const cv::Mat& image) {
        cv::Mat gray;

        if (image.channels() == 3) {
            cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
        } else {
            gray = image.clone();
        }

        cv::Scalar mean, stddev;
        cv::meanStdDev(gray, mean, stddev);

        // Uniformity is inversely related to standard deviation
        float coefficient_of_variation = static_cast<float>(stddev[0] / mean[0]);
        return std::max(0.0f, 1.0f - coefficient_of_variation);
    }

    /**
     * @brief Compute image contrast
     */
    float computeImageContrast(const cv::Mat& image) {
        cv::Mat gray;

        if (image.channels() == 3) {
            cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
        } else {
            gray = image.clone();
        }

        cv::Scalar mean, stddev;
        cv::meanStdDev(gray, mean, stddev);

        // Contrast based on standard deviation normalized by mean
        float contrast = static_cast<float>(stddev[0] / (mean[0] + 1e-6));
        return std::min(1.0f, contrast / 0.5f); // Normalize to 0-1 range
    }

    /**
     * @brief Estimate head pose from landmarks
     */
    cv::Vec3f estimateHeadPose(const FacialLandmarks& landmarks) {
        if (landmarks.points2D.size() < 34) {
            return cv::Vec3f(0, 0, 0);
        }

        // Use key facial points for pose estimation
        const auto& nose_tip = landmarks.points2D[33];     // Nose tip
        const auto& left_eye = landmarks.points2D[36];     // Left eye
        const auto& right_eye = landmarks.points2D[45];    // Right eye
        const auto& mouth_left = landmarks.points2D[48];   // Left mouth corner
        const auto& mouth_right = landmarks.points2D[54];  // Right mouth corner

        // Estimate yaw from eye-nose-mouth asymmetry
        float eye_center_x = (left_eye.x + right_eye.x) * 0.5f;
        float mouth_center_x = (mouth_left.x + mouth_right.x) * 0.5f;

        float yaw_offset = nose_tip.x - eye_center_x;
        float yaw_angle = std::atan2(yaw_offset, 100.0f) * 180.0f / M_PI; // Convert to degrees

        // Estimate pitch from vertical alignment
        float eye_center_y = (left_eye.y + right_eye.y) * 0.5f;
        float pitch_offset = nose_tip.y - eye_center_y;
        float pitch_angle = std::atan2(pitch_offset, 50.0f) * 180.0f / M_PI;

        // Estimate roll from eye alignment
        float eye_slope = (right_eye.y - left_eye.y) / (right_eye.x - left_eye.x + 1e-6f);
        float roll_angle = std::atan(eye_slope) * 180.0f / M_PI;

        return cv::Vec3f(yaw_angle, pitch_angle, roll_angle);
    }

    /**
     * @brief Assess facial expression neutrality
     */
    float assessExpressionNeutrality(const FacialLandmarks& landmarks) {
        if (landmarks.points2D.size() < 68) return 0.5f;

        float neutrality_score = 1.0f;

        // Check mouth expression
        const auto& mouth_left = landmarks.points2D[48];
        const auto& mouth_right = landmarks.points2D[54];
        const auto& mouth_top = landmarks.points2D[51];
        const auto& mouth_bottom = landmarks.points2D[57];

        float mouth_width = std::abs(mouth_right.x - mouth_left.x);
        float mouth_height = std::abs(mouth_bottom.y - mouth_top.y);
        float mouth_aspect_ratio = mouth_height / (mouth_width + 1e-6f);

        // Neutral mouth has specific aspect ratio
        if (mouth_aspect_ratio > 0.3f) { // Too open (surprise/shock)
            neutrality_score *= 0.6f;
        } else if (mouth_aspect_ratio < 0.1f) { // Too closed or compressed
            neutrality_score *= 0.8f;
        }

        // Check eye expression
        const auto& left_eye_top = landmarks.points2D[37];
        const auto& left_eye_bottom = landmarks.points2D[41];
        const auto& right_eye_top = landmarks.points2D[44];
        const auto& right_eye_bottom = landmarks.points2D[46];

        float left_eye_openness = std::abs(left_eye_bottom.y - left_eye_top.y);
        float right_eye_openness = std::abs(right_eye_bottom.y - right_eye_top.y);

        // Check for asymmetric eye opening (winking)
        float eye_asymmetry = std::abs(left_eye_openness - right_eye_openness);
        if (eye_asymmetry > 5.0f) {
            neutrality_score *= 0.7f;
        }

        // Check eyebrow position for emotional expression
        const auto& left_brow_inner = landmarks.points2D[22];
        const auto& left_brow_outer = landmarks.points2D[26];
        const auto& left_eye_center = landmarks.points2D[39];

        float brow_height = left_eye_center.y - (left_brow_inner.y + left_brow_outer.y) * 0.5f;
        if (brow_height < 10.0f) { // Frowning
            neutrality_score *= 0.7f;
        } else if (brow_height > 25.0f) { // Surprised
            neutrality_score *= 0.8f;
        }

        return std::max(0.0f, std::min(1.0f, neutrality_score));
    }

    /**
     * @brief Validate template encryption integrity
     */
    bool validateTemplateEncryption(const BiometricTemplate& template_data) {
        // Check if encrypted data is present
        if (template_data.encrypted_features.empty()) {
            return false;
        }

        // Validate encryption parameters
        if (template_data.initialization_vector.size() != 16) {
            return false; // Invalid IV size for AES
        }

        if (template_data.encryption_key_hash.size() != 32) {
            return false; // Invalid key hash size for SHA-256
        }

        // Check encrypted data size is multiple of AES block size
        if (template_data.encrypted_features.size() % 16 != 0) {
            return false;
        }

        // Validate template hash integrity
        // In a real implementation, this would recompute and compare the hash
        bool hash_valid = !std::all_of(template_data.template_hash.begin(),
                                      template_data.template_hash.end(),
                                      [](uint8_t b) { return b == 0; });

        return hash_valid;
    }

    /**
     * @brief Validate template integrity
     */
    bool validateTemplateIntegrity(const BiometricTemplate& template_data) {
        // Basic integrity checks
        if (template_data.template_id.empty() || template_data.subject_id.empty()) {
            return false;
        }

        if (template_data.template_version == 0) {
            return false;
        }

        if (template_data.feature_count == 0 || template_data.landmark_count == 0) {
            return false;
        }

        // Check timestamp validity
        auto now = std::chrono::system_clock::now();
        if (template_data.creation_time > now) {
            return false; // Future timestamp not valid
        }

        // Check if template is not too old (7 years for banking)
        auto max_age = std::chrono::hours(24 * banking_requirements_.max_template_retention_days);
        if (now - template_data.creation_time > max_age) {
            return false;
        }

        return true;
    }

    /**
     * @brief Validate GDPR compliance
     */
    bool validateGDPRCompliance(const BiometricTemplate& template_data) {
        // Check data minimization - only essential data should be present
        if (template_data.encrypted_features.empty()) {
            return false; // No biometric data
        }

        // Check if creation device is recorded (for audit)
        if (template_data.creation_device_id.empty()) {
            return false;
        }

        // Check encryption algorithm is specified
        if (template_data.encryption_algorithm.empty()) {
            return false;
        }

        // Additional GDPR checks would be implemented here
        return true;
    }

    /**
     * @brief Generate compliance recommendations
     */
    void generateComplianceRecommendations(ComplianceResult& result) {
        result.recommendations.clear();

        // ISO compliance recommendations
        if (!result.iso_19794_5_compliant) {
            if (result.detailed_metrics.count("face_width_pixels") &&
                result.detailed_metrics["face_width_pixels"] < iso_requirements_.min_face_width_pixels) {
                result.recommendations.push_back("Increase face size in image or move camera closer");
            }

            if (result.detailed_metrics.count("image_sharpness") &&
                result.detailed_metrics["image_sharpness"] < iso_requirements_.min_sharpness_score) {
                result.recommendations.push_back("Improve image focus and reduce motion blur");
            }

            if (result.detailed_metrics.count("illumination_uniformity") &&
                result.detailed_metrics["illumination_uniformity"] < iso_requirements_.min_illumination_uniformity) {
                result.recommendations.push_back("Improve lighting uniformity and reduce shadows");
            }
        }

        // Banking compliance recommendations
        if (!result.banking_compliant) {
            if (result.detailed_metrics.count("false_accept_rate") &&
                result.detailed_metrics["false_accept_rate"] > banking_requirements_.max_false_accept_rate) {
                result.recommendations.push_back("Increase matching threshold to reduce false accepts");
            }

            if (result.detailed_metrics.count("template_quality") &&
                result.detailed_metrics["template_quality"] < banking_requirements_.min_template_quality) {
                result.recommendations.push_back("Improve image quality during enrollment");
            }

            if (result.detailed_metrics.count("encryption_validated") &&
                result.detailed_metrics["encryption_validated"] < 1.0f) {
                result.recommendations.push_back("Verify template encryption implementation");
            }
        }

        // General recommendations
        if (result.compliance_score < 0.95f) {
            result.recommendations.push_back("Review and address all compliance violations");
            result.recommendations.push_back("Consider system recalibration or algorithm updates");
        }
    }

    /**
     * @brief Generate unique validation ID
     */
    std::string generateValidationId() {
        auto now = std::chrono::system_clock::now();
        auto time_t = std::chrono::system_clock::to_time_t(now);
        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;

        std::stringstream ss;
        ss << "VAL_" << std::put_time(std::gmtime(&time_t), "%Y%m%d_%H%M%S")
           << "_" << std::setfill('0') << std::setw(3) << ms.count();

        return ss.str();
    }

    /**
     * @brief Format timestamp for reporting
     */
    std::string formatTimestamp(const std::chrono::system_clock::time_point& timestamp) {
        auto time_t = std::chrono::system_clock::to_time_t(timestamp);
        std::stringstream ss;
        ss << std::put_time(std::gmtime(&time_t), "%Y-%m-%d %H:%M:%S UTC");
        return ss.str();
    }
};

} // namespace face
} // namespace unlook