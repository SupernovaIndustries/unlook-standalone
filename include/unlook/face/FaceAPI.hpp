#pragma once

/**
 * @file FaceAPI.hpp
 * @brief Main header for Unlook Banking-Grade Facial Recognition API
 *
 * Complete facial recognition system designed for banking applications
 * with industrial-grade precision and security compliance.
 *
 * @version 1.0.0-banking
 * @author Unlook Team
 * @copyright 2025 Unlook. All rights reserved.
 */

// Core face recognition types and definitions
#include "unlook/face/FaceTypes.hpp"

// Face detection and analysis
#include "unlook/face/FaceDetector.hpp"
#include "unlook/face/LandmarkExtractor.hpp"
#include "unlook/face/Face3DReconstructor.hpp"

// Biometric template generation and matching
#include "unlook/face/FaceEncoder.hpp"
#include "unlook/face/FaceMatcher.hpp"

// Security and anti-spoofing
#include "unlook/face/LivenessDetector.hpp"

// Enrollment and user experience
#include "unlook/face/FaceEnroller.hpp"

// Cloud integration
#include "unlook/face/SupernovaMLInterface.hpp"

// Core Unlook integration
#include "unlook/core/types.hpp"
#include "unlook/core/Logger.hpp"
#include "unlook/stereo/DepthProcessor.hpp"
#include "unlook/calibration/CalibrationManager.hpp"

#include <memory>
#include <string>
#include <vector>
#include <functional>

namespace unlook {

/**
 * @brief Main facial recognition API namespace
 *
 * Banking-grade facial recognition system with industrial precision
 * and comprehensive security features for financial applications.
 */
namespace face {

/**
 * @brief Complete facial recognition system configuration
 */
struct FaceRecognitionSystemConfig {
    // Core system settings
    bool enable_face_detection = true;       ///< Enable face detection
    bool enable_landmark_extraction = true;  ///< Enable landmark extraction
    bool enable_3d_reconstruction = true;    ///< Enable 3D face reconstruction
    bool enable_liveness_detection = true;   ///< Enable liveness detection
    bool enable_template_generation = true;  ///< Enable template generation
    bool enable_face_matching = true;        ///< Enable face matching
    bool enable_supernova_integration = false; ///< Enable Supernova ML

    // Component configurations
    FaceDetectionConfig detector_config;     ///< Face detector configuration
    LandmarkExtractionConfig landmark_config; ///< Landmark extractor configuration
    Face3DReconstructionConfig reconstruction_config; ///< 3D reconstructor configuration
    FaceEncodingConfig encoding_config;      ///< Face encoder configuration
    FaceMatchingConfig matching_config;      ///< Face matcher configuration
    LivenessDetectionConfig liveness_config; ///< Liveness detector configuration
    EnrollmentConfig enrollment_config;      ///< Enrollment system configuration
    SupernovaMLConfig supernova_config;      ///< Supernova ML configuration

    // Banking compliance settings
    bool banking_grade_mode = false;         ///< Enable banking grade mode
    float target_far = 0.0001f;             ///< Target false accept rate (0.01%)
    float target_frr = 0.03f;               ///< Target false reject rate (3%)
    bool iso_compliance_required = false;    ///< Require ISO/IEC 19794-5 compliance
    bool comprehensive_audit_trail = false;  ///< Enable comprehensive audit trail

    // Performance settings
    bool enable_gpu_acceleration = false;    ///< Enable GPU acceleration
    bool enable_multithreading = true;       ///< Enable multithreading
    int max_processing_threads = 4;          ///< Maximum processing threads
    bool enable_performance_monitoring = true; ///< Enable performance monitoring

    bool validate() const;
    std::string toString() const;
};

/**
 * @brief Complete facial recognition system performance metrics
 */
struct FaceRecognitionMetrics {
    // Detection metrics
    double avg_detection_time_ms = 0.0;      ///< Average face detection time
    size_t total_detections = 0;             ///< Total faces detected
    float detection_success_rate = 0.0f;     ///< Detection success rate

    // Recognition metrics
    double avg_recognition_time_ms = 0.0;    ///< Average recognition time
    size_t total_recognitions = 0;           ///< Total recognition attempts
    float recognition_accuracy = 0.0f;       ///< Overall recognition accuracy

    // Quality metrics
    float avg_face_quality = 0.0f;          ///< Average face quality score
    float avg_template_quality = 0.0f;       ///< Average template quality score
    float avg_liveness_confidence = 0.0f;    ///< Average liveness confidence

    // Security metrics
    size_t presentation_attacks_blocked = 0;  ///< Presentation attacks blocked
    float liveness_success_rate = 0.0f;      ///< Liveness detection success rate
    size_t security_violations = 0;          ///< Security policy violations

    // Performance metrics
    double system_uptime_hours = 0.0;        ///< System uptime
    float memory_usage_mb = 0.0f;            ///< Current memory usage
    float cpu_usage_percent = 0.0f;          ///< Current CPU usage

    std::string toString() const;
};

/**
 * @brief Banking-Grade Facial Recognition System
 *
 * Complete facial recognition system integrating all components for
 * banking-grade authentication with industrial precision and security.
 *
 * Features:
 * - End-to-end facial recognition pipeline
 * - Banking compliance (FAR < 0.001%, FRR < 3%)
 * - Multi-modal liveness detection
 * - 3D face reconstruction with sub-millimeter precision
 * - Encrypted template storage and matching
 * - Comprehensive audit trail and compliance reporting
 * - Intel RealSense style enrollment experience
 * - Supernova ML cloud integration
 * - ARM64/CM4/CM5 optimized performance
 *
 * Integrates seamlessly with Unlook stereo vision system.
 */
class FaceRecognitionSystem {
public:
    /**
     * @brief Constructor with default configuration
     */
    FaceRecognitionSystem();

    /**
     * @brief Constructor with custom configuration
     * @param config System configuration
     */
    explicit FaceRecognitionSystem(const FaceRecognitionSystemConfig& config);

    /**
     * @brief Destructor
     */
    ~FaceRecognitionSystem();

    /**
     * @brief Initialize complete facial recognition system
     * @param calibration_manager Stereo calibration manager
     * @param depth_processor Depth processing system
     * @param config System configuration
     * @return FaceResultCode indicating initialization result
     */
    FaceResultCode initialize(
        std::shared_ptr<calibration::CalibrationManager> calibration_manager,
        std::shared_ptr<stereo::DepthProcessor> depth_processor,
        const FaceRecognitionSystemConfig& config);

    /**
     * @brief Check if system is fully initialized
     * @return True if all components are ready
     */
    bool isInitialized() const;

    /**
     * @brief Perform complete face recognition from stereo images
     * @param left_image Left stereo image (CV_8UC3)
     * @param right_image Right stereo image (CV_8UC3)
     * @param reference_templates Vector of reference templates for matching
     * @param recognition_results Output recognition results
     * @return FaceResultCode indicating processing result
     */
    FaceResultCode recognizeFace(const cv::Mat& left_image,
                                const cv::Mat& right_image,
                                const std::vector<BiometricTemplate>& reference_templates,
                                std::vector<FaceMatchResult>& recognition_results);

    /**
     * @brief Perform banking-grade face verification
     * @param left_image Left stereo image
     * @param right_image Right stereo image
     * @param reference_template Reference template for verification
     * @param transaction_context Banking transaction context
     * @param verification_result Output verification result
     * @return FaceResultCode indicating verification result
     */
    FaceResultCode verifyFaceBankingGrade(
        const cv::Mat& left_image,
        const cv::Mat& right_image,
        const BiometricTemplate& reference_template,
        const std::map<std::string, std::string>& transaction_context,
        FaceMatchResult& verification_result);

    /**
     * @brief Start interactive face enrollment session
     * @param user_id User identifier for enrollment
     * @param enrollment_config Enrollment configuration
     * @param progress_callback Progress update callback
     * @return FaceResultCode indicating enrollment start result
     */
    FaceResultCode startFaceEnrollment(
        const std::string& user_id,
        const EnrollmentConfig& enrollment_config,
        EnrollmentProgressCallback progress_callback);

    /**
     * @brief Process enrollment frame
     * @param left_image Left stereo image
     * @param right_image Right stereo image
     * @param enrollment_preview Output preview image with guidance
     * @return FaceResultCode indicating frame processing result
     */
    FaceResultCode processEnrollmentFrame(const cv::Mat& left_image,
                                         const cv::Mat& right_image,
                                         cv::Mat& enrollment_preview);

    /**
     * @brief Complete enrollment and generate template
     * @param final_template Output generated biometric template
     * @return FaceResultCode indicating enrollment completion result
     */
    FaceResultCode completeEnrollment(BiometricTemplate& final_template);

    /**
     * @brief Extract facial features for analysis
     * @param rgb_image Input RGB image
     * @param depth_map Input depth map
     * @param face_analysis Output facial analysis results
     * @return FaceResultCode indicating analysis result
     */
    FaceResultCode analyzeFace(const cv::Mat& rgb_image,
                              const cv::Mat& depth_map,
                              struct FacialAnalysis& face_analysis);

    /**
     * @brief Generate 3D face model from stereo images
     * @param left_image Left stereo image
     * @param right_image Right stereo image
     * @param face_model Output 3D face model
     * @return FaceResultCode indicating reconstruction result
     */
    FaceResultCode generate3DFaceModel(const cv::Mat& left_image,
                                      const cv::Mat& right_image,
                                      Face3DModel& face_model);

    /**
     * @brief Perform comprehensive liveness detection
     * @param rgb_frames Sequence of RGB frames
     * @param depth_frames Corresponding depth frames
     * @param liveness_result Output liveness detection result
     * @return FaceResultCode indicating liveness detection result
     */
    FaceResultCode detectLiveness(const std::vector<cv::Mat>& rgb_frames,
                                 const std::vector<cv::Mat>& depth_frames,
                                 LivenessResult& liveness_result);

    /**
     * @brief Get individual system components for advanced use
     * @param detector Output face detector component
     * @param landmark_extractor Output landmark extractor component
     * @param reconstructor Output 3D reconstructor component
     * @param encoder Output face encoder component
     * @param matcher Output face matcher component
     * @param liveness_detector Output liveness detector component
     * @param enroller Output face enroller component
     */
    void getComponents(std::shared_ptr<FaceDetector>& detector,
                      std::shared_ptr<LandmarkExtractor>& landmark_extractor,
                      std::shared_ptr<Face3DReconstructor>& reconstructor,
                      std::shared_ptr<FaceEncoder>& encoder,
                      std::shared_ptr<FaceMatcher>& matcher,
                      std::shared_ptr<LivenessDetector>& liveness_detector,
                      std::shared_ptr<FaceEnroller>& enroller);

    /**
     * @brief Enable Supernova ML integration
     * @param supernova_config Supernova ML configuration
     * @return FaceResultCode indicating integration result
     */
    FaceResultCode enableSupernovaML(const SupernovaMLConfig& supernova_config);

    /**
     * @brief Set system configuration
     * @param config New system configuration
     * @return FaceResultCode indicating configuration result
     */
    FaceResultCode setConfiguration(const FaceRecognitionSystemConfig& config);

    /**
     * @brief Get current system configuration
     * @return Current system configuration
     */
    FaceRecognitionSystemConfig getConfiguration() const;

    /**
     * @brief Get comprehensive system metrics
     * @param metrics Output system performance metrics
     * @return FaceResultCode indicating metrics retrieval result
     */
    FaceResultCode getSystemMetrics(FaceRecognitionMetrics& metrics) const;

    /**
     * @brief Reset all system statistics
     */
    void resetSystemStatistics();

    /**
     * @brief Export comprehensive system audit log
     * @param output_path Path to export audit log
     * @param format Export format ("json", "csv", "xml")
     * @param include_templates Include template metadata
     * @return FaceResultCode indicating export result
     */
    FaceResultCode exportSystemAuditLog(const std::string& output_path,
                                       const std::string& format = "json",
                                       bool include_templates = false);

    /**
     * @brief Perform system self-diagnostics
     * @param diagnostic_results Output diagnostic results
     * @return FaceResultCode indicating diagnostic result
     */
    FaceResultCode runSystemDiagnostics(std::map<std::string, std::string>& diagnostic_results);

    /**
     * @brief Get system health status
     * @param health_status Output health status information
     * @return FaceResultCode indicating health check result
     */
    FaceResultCode getSystemHealth(std::map<std::string, float>& health_status);

    /**
     * @brief Enable banking-grade mode for all components
     * @param enable Enable/disable banking grade mode
     * @param compliance_level Required compliance level
     * @param audit_level Audit logging level
     */
    void setBankingGradeMode(bool enable,
                            const std::string& compliance_level = "PCI_DSS",
                            const std::string& audit_level = "COMPREHENSIVE");

    /**
     * @brief Get last error message from any component
     * @return Human-readable error message
     */
    std::string getLastError() const;

private:
    // System components
    std::shared_ptr<FaceDetector> face_detector_;
    std::shared_ptr<LandmarkExtractor> landmark_extractor_;
    std::shared_ptr<Face3DReconstructor> face_reconstructor_;
    std::shared_ptr<FaceEncoder> face_encoder_;
    std::shared_ptr<FaceMatcher> face_matcher_;
    std::shared_ptr<LivenessDetector> liveness_detector_;
    std::shared_ptr<FaceEnroller> face_enroller_;
    std::shared_ptr<SupernovaMLInterface> supernova_interface_;

    // Integration components
    std::shared_ptr<calibration::CalibrationManager> calibration_manager_;
    std::shared_ptr<stereo::DepthProcessor> depth_processor_;

    // System state
    FaceRecognitionSystemConfig config_;
    mutable std::mutex config_mutex_;
    std::atomic<bool> initialized_{false};
    std::atomic<bool> banking_grade_enabled_{false};

    // Performance monitoring
    mutable std::mutex metrics_mutex_;
    FaceRecognitionMetrics system_metrics_;
    std::chrono::system_clock::time_point start_time_;

    // Error handling
    mutable std::mutex error_mutex_;
    std::string last_error_message_;
    std::vector<std::string> error_history_;

    /**
     * @brief Initialize individual components
     * @return FaceResultCode indicating component initialization result
     */
    FaceResultCode initializeComponents();

    /**
     * @brief Update system metrics
     * @param component_name Name of component
     * @param processing_time Processing time
     * @param success Whether operation was successful
     */
    void updateMetrics(const std::string& component_name,
                      double processing_time,
                      bool success);

    /**
     * @brief Log system event for audit trail
     * @param event_type Type of event
     * @param event_data Event data
     * @param user_id Associated user ID (if applicable)
     */
    void logSystemEvent(const std::string& event_type,
                       const std::map<std::string, std::string>& event_data,
                       const std::string& user_id = "");

    /**
     * @brief Validate system health
     * @return True if all components are healthy
     */
    bool validateSystemHealth();

    /**
     * @brief Set error message for system
     * @param error_message Error message to set
     */
    void setLastError(const std::string& error_message);

    // Disable copy constructor and assignment
    FaceRecognitionSystem(const FaceRecognitionSystem&) = delete;
    FaceRecognitionSystem& operator=(const FaceRecognitionSystem&) = delete;

    // Enable move semantics
    FaceRecognitionSystem(FaceRecognitionSystem&&) noexcept;
    FaceRecognitionSystem& operator=(FaceRecognitionSystem&&) noexcept;
};

/**
 * @brief Additional facial analysis result structure
 */
struct FacialAnalysis {
    FaceDetectionResult detection;           ///< Face detection results
    FacialLandmarks landmarks;               ///< Facial landmarks
    Face3DModel face_model;                  ///< 3D face model
    LivenessResult liveness;                 ///< Liveness detection result
    BiometricTemplate face_template;         ///< Generated biometric template

    // Quality assessments
    float overall_quality_score = 0.0f;     ///< Overall analysis quality
    float authentication_confidence = 0.0f;  ///< Confidence for authentication
    bool banking_grade_ready = false;       ///< Ready for banking use

    // Processing metrics
    double total_processing_time_ms = 0.0;   ///< Total processing time
    std::map<std::string, double> component_times; ///< Per-component timing

    FacialAnalysis() = default;
};

/**
 * @brief System-wide utility functions
 */
class FaceRecognitionUtils {
public:
    /**
     * @brief Validate complete system configuration
     * @param config Configuration to validate
     * @param validation_errors Output validation errors
     * @return True if configuration is valid
     */
    static bool validateSystemConfiguration(
        const FaceRecognitionSystemConfig& config,
        std::vector<std::string>& validation_errors);

    /**
     * @brief Get recommended configuration for banking applications
     * @param target_far Target false accept rate
     * @param target_frr Target false reject rate
     * @return Recommended banking-grade configuration
     */
    static FaceRecognitionSystemConfig getBankingGradeConfiguration(
        float target_far = 0.0001f,
        float target_frr = 0.03f);

    /**
     * @brief Generate system performance report
     * @param metrics System metrics
     * @param include_recommendations Include performance recommendations
     * @return Formatted performance report
     */
    static std::string generatePerformanceReport(
        const FaceRecognitionMetrics& metrics,
        bool include_recommendations = true);

    /**
     * @brief Estimate system resource requirements
     * @param config System configuration
     * @param expected_load Expected system load
     * @return Resource requirement estimates
     */
    static std::map<std::string, std::string> estimateResourceRequirements(
        const FaceRecognitionSystemConfig& config,
        const std::map<std::string, float>& expected_load);
};

} // namespace face

/**
 * @brief Quick-start factory functions for common use cases
 */
namespace quickstart {

/**
 * @brief Create banking-grade facial recognition system
 * @param calibration_manager Stereo calibration manager
 * @param depth_processor Depth processor
 * @return Configured banking-grade system
 */
std::unique_ptr<face::FaceRecognitionSystem> createBankingGradeSystem(
    std::shared_ptr<calibration::CalibrationManager> calibration_manager,
    std::shared_ptr<stereo::DepthProcessor> depth_processor);

/**
 * @brief Create standard facial recognition system
 * @param calibration_manager Stereo calibration manager
 * @param depth_processor Depth processor
 * @return Configured standard system
 */
std::unique_ptr<face::FaceRecognitionSystem> createStandardSystem(
    std::shared_ptr<calibration::CalibrationManager> calibration_manager,
    std::shared_ptr<stereo::DepthProcessor> depth_processor);

/**
 * @brief Create enrollment-only system
 * @param calibration_manager Stereo calibration manager
 * @param depth_processor Depth processor
 * @return Configured enrollment system
 */
std::unique_ptr<face::FaceRecognitionSystem> createEnrollmentSystem(
    std::shared_ptr<calibration::CalibrationManager> calibration_manager,
    std::shared_ptr<stereo::DepthProcessor> depth_processor);

} // namespace quickstart
} // namespace unlook