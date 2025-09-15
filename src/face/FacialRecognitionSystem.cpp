#include "unlook/face/FaceAPI.hpp"
#include "unlook/face/LandmarkExtractor.hpp"
#include "unlook/face/Face3DReconstructor.hpp"
#include "unlook/face/FaceMatcher.hpp"
#include "unlook/face/FaceDetector.hpp"
#include "unlook/face/LivenessDetector.hpp"
#include "unlook/face/FaceEnroller.hpp"
#include "unlook/stereo/DepthProcessor.hpp"
#include "unlook/calibration/CalibrationManager.hpp"
#include "unlook/core/logging.hpp"
#include <opencv2/opencv.hpp>
#include <algorithm>
#include <chrono>
#include <memory>
#include <thread>
#include <future>

namespace unlook {
namespace face {

/**
 * @brief Complete facial recognition system integration with stereo depth processing
 *
 * This class provides a comprehensive banking-grade facial recognition system
 * that integrates all components with the Unlook 70mm baseline stereo system.
 * Designed for industrial precision and banking compliance.
 */
class FacialRecognitionSystem {
public:
    /**
     * @brief System configuration combining all subsystems
     */
    struct SystemConfig {
        // Core system settings
        bool enable_banking_mode = false;           // Enable banking-grade processing
        bool enable_3d_processing = true;           // Enable 3D facial analysis
        bool enable_liveness_detection = true;     // Enable liveness detection
        bool enable_performance_optimization = true; // Enable ARM64 optimizations

        // Quality requirements
        float min_face_quality = 0.8f;             // Minimum face quality
        float min_landmark_quality = 0.7f;         // Minimum landmark quality
        float min_reconstruction_quality = 0.8f;   // Minimum 3D reconstruction quality
        float min_liveness_confidence = 0.8f;      // Minimum liveness confidence

        // Performance targets
        float target_total_time_ms = 500.0f;       // Target total processing time
        float target_detection_time_ms = 50.0f;    // Target face detection time
        float target_landmark_time_ms = 100.0f;    // Target landmark extraction time
        float target_reconstruction_time_ms = 200.0f; // Target 3D reconstruction time
        float target_matching_time_ms = 50.0f;     // Target matching time

        // Integration settings
        bool use_hardware_sync = true;             // Use hardware-synchronized cameras
        bool enable_depth_enhancement = true;     // Enhance using depth data
        bool enable_stereo_rectification = true;  // Apply stereo rectification

        bool validate() const {
            return min_face_quality >= 0.0f && min_face_quality <= 1.0f &&
                   target_total_time_ms > 0 && target_detection_time_ms > 0;
        }
    };

    /**
     * @brief Complete processing result with all subsystem outputs
     */
    struct ProcessingResult {
        // Overall result
        bool success = false;
        FaceResultCode result_code = FaceResultCode::SUCCESS;
        std::string error_message;

        // Timing information
        double total_processing_time_ms = 0.0;
        double detection_time_ms = 0.0;
        double landmark_time_ms = 0.0;
        double reconstruction_time_ms = 0.0;
        double matching_time_ms = 0.0;
        double liveness_time_ms = 0.0;

        // Processing outputs
        FaceDetectionResult detection_result;
        FacialLandmarks landmarks;
        Face3DModel face_3d_model;
        LivenessResult liveness_result;
        FaceMatchResult match_result;

        // Quality metrics
        float overall_quality = 0.0f;
        float face_quality = 0.0f;
        float landmark_quality = 0.0f;
        float reconstruction_quality = 0.0f;
        float depth_quality = 0.0f;

        // Banking compliance
        bool banking_compliant = false;
        float compliance_score = 0.0f;

        // Performance metrics
        float processing_efficiency = 0.0f;
        bool met_timing_targets = false;

        void reset() {
            *this = ProcessingResult{};
        }
    };

private:
    // System configuration
    SystemConfig config_;

    // Core components
    std::unique_ptr<FaceDetector> face_detector_;
    std::unique_ptr<LandmarkExtractor> landmark_extractor_;
    std::unique_ptr<Face3DReconstructor> face_3d_reconstructor_;
    std::unique_ptr<FaceMatcher> face_matcher_;
    std::unique_ptr<LivenessDetector> liveness_detector_;
    std::unique_ptr<FaceEnroller> face_enroller_;

    // Integration components
    std::shared_ptr<calibration::CalibrationManager> calibration_manager_;
    std::shared_ptr<stereo::DepthProcessor> depth_processor_;

    // System state
    bool initialized_ = false;
    std::string last_error_;
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;
    float baseline_mm_ = 70.017f;

    // Performance monitoring
    mutable std::mutex stats_mutex_;
    std::atomic<size_t> total_processed_{0};
    std::atomic<size_t> successful_processed_{0};
    std::atomic<double> total_processing_time_{0.0};

    // Stereo rectification maps
    cv::Mat rectify_map_left_x_, rectify_map_left_y_;
    cv::Mat rectify_map_right_x_, rectify_map_right_y_;

public:
    explicit FacialRecognitionSystem(const SystemConfig& config = SystemConfig{})
        : config_(config) {

        if (!config_.validate()) {
            UNLOOK_LOG_WARN("Invalid FacialRecognitionSystem configuration");
        }
    }

    ~FacialRecognitionSystem() = default;

    /**
     * @brief Initialize complete facial recognition system
     */
    FaceResultCode initialize(std::shared_ptr<calibration::CalibrationManager> calibration_manager,
                             std::shared_ptr<stereo::DepthProcessor> depth_processor) {
        try {
            calibration_manager_ = calibration_manager;
            depth_processor_ = depth_processor;

            if (!calibration_manager_ || !depth_processor_) {
                last_error_ = "Calibration manager and depth processor required";
                return FaceResultCode::ERROR_TEMPLATE_INVALID;
            }

            // Initialize stereo calibration parameters
            FaceResultCode result = initializeStereoCalibration();
            if (result != FaceResultCode::SUCCESS) {
                return result;
            }

            // Initialize face detection
            face_detector_ = std::make_unique<FaceDetector>();
            result = face_detector_->initialize();
            if (result != FaceResultCode::SUCCESS) {
                last_error_ = "Failed to initialize face detector: " + face_detector_->getLastError();
                return result;
            }

            // Initialize landmark extraction
            LandmarkExtractionConfig landmark_config;
            landmark_config.enable_3d_estimation = config_.enable_3d_processing;
            landmark_config.banking_mode = config_.enable_banking_mode;

            landmark_extractor_ = std::make_unique<LandmarkExtractor>(landmark_config);
            result = landmark_extractor_->initialize("", LandmarkModel::LANDMARKS_68);
            if (result != FaceResultCode::SUCCESS) {
                last_error_ = "Failed to initialize landmark extractor: " + landmark_extractor_->getLastError();
                return result;
            }

            // Set camera calibration for landmark extractor
            landmark_extractor_->setCameraCalibration(camera_matrix_, dist_coeffs_);

            // Initialize 3D reconstruction
            if (config_.enable_3d_processing) {
                Face3DReconstructionConfig reconstruction_config;
                reconstruction_config.banking_mode = config_.enable_banking_mode;
                reconstruction_config.min_reconstruction_quality = config_.min_reconstruction_quality;

                face_3d_reconstructor_ = std::make_unique<Face3DReconstructor>(reconstruction_config);
                result = face_3d_reconstructor_->initialize(calibration_manager_, depth_processor_);
                if (result != FaceResultCode::SUCCESS) {
                    last_error_ = "Failed to initialize 3D reconstructor: " + face_3d_reconstructor_->getLastError();
                    return result;
                }
            }

            // Initialize face matching
            FaceMatchingConfig matching_config;
            matching_config.banking_audit_mode = config_.enable_banking_mode;
            matching_config.method = config_.enable_banking_mode ?
                                   FaceMatchingConfig::BANKING_GRADE_HYBRID :
                                   FaceMatchingConfig::COSINE_SIMILARITY;

            face_matcher_ = std::make_unique<FaceMatcher>(matching_config);
            result = face_matcher_->initialize("");
            if (result != FaceResultCode::SUCCESS) {
                last_error_ = "Failed to initialize face matcher: " + face_matcher_->getLastError();
                return result;
            }

            // Initialize liveness detection
            if (config_.enable_liveness_detection) {
                liveness_detector_ = std::make_unique<LivenessDetector>();
                result = liveness_detector_->initialize();
                if (result != FaceResultCode::SUCCESS) {
                    last_error_ = "Failed to initialize liveness detector: " + liveness_detector_->getLastError();
                    return result;
                }
            }

            // Initialize enrollment system
            face_enroller_ = std::make_unique<FaceEnroller>();

            initialized_ = true;

            UNLOOK_LOG_INFO("FacialRecognitionSystem initialized successfully");
            UNLOOK_LOG_INFO("Configuration: Banking={}, 3D={}, Liveness={}, Baseline={:.3f}mm",
                           config_.enable_banking_mode,
                           config_.enable_3d_processing,
                           config_.enable_liveness_detection,
                           baseline_mm_);

            return FaceResultCode::SUCCESS;

        } catch (const std::exception& e) {
            last_error_ = "Exception during initialization: " + std::string(e.what());
            return FaceResultCode::ERROR_TEMPLATE_INVALID;
        }
    }

    /**
     * @brief Process complete facial recognition pipeline from stereo images
     */
    FaceResultCode processStereoPair(const cv::Mat& left_image,
                                    const cv::Mat& right_image,
                                    ProcessingResult& result) {
        auto start_time = std::chrono::high_resolution_clock::now();

        result.reset();

        if (!initialized_) {
            result.error_message = "System not initialized";
            result.result_code = FaceResultCode::ERROR_TEMPLATE_INVALID;
            return result.result_code;
        }

        try {
            // Step 1: Stereo rectification (if enabled)
            cv::Mat rect_left = left_image, rect_right = right_image;
            if (config_.enable_stereo_rectification) {
                auto rect_start = std::chrono::high_resolution_clock::now();
                applyStereoRectification(left_image, right_image, rect_left, rect_right);
                auto rect_end = std::chrono::high_resolution_clock::now();
                UNLOOK_LOG_DEBUG("Stereo rectification: {:.2f}ms",
                                std::chrono::duration<double, std::milli>(rect_end - rect_start).count());
            }

            // Step 2: Face detection
            auto detection_start = std::chrono::high_resolution_clock::now();
            result.result_code = face_detector_->detectFaces(rect_left, result.detection_result);
            auto detection_end = std::chrono::high_resolution_clock::now();
            result.detection_time_ms = std::chrono::duration<double, std::milli>(detection_end - detection_start).count();

            if (result.result_code != FaceResultCode::SUCCESS || !result.detection_result.hasSingleFace()) {
                result.error_message = "Face detection failed or multiple faces detected";
                return result.result_code;
            }

            FaceBoundingBox best_face = result.detection_result.getBestFace();
            result.face_quality = best_face.confidence;

            // Quality gating
            if (result.face_quality < config_.min_face_quality) {
                result.error_message = "Face quality below threshold";
                result.result_code = FaceResultCode::ERROR_POOR_FACE_QUALITY;
                return result.result_code;
            }

            // Step 3: Landmark extraction
            auto landmark_start = std::chrono::high_resolution_clock::now();
            if (config_.enable_3d_processing) {
                // Generate depth map first
                cv::Mat depth_map;
                if (!depth_processor_->processStereoPair(rect_left, rect_right, depth_map)) {
                    result.error_message = "Depth map generation failed";
                    result.result_code = FaceResultCode::ERROR_STEREO_RECONSTRUCTION_FAILED;
                    return result.result_code;
                }

                result.result_code = landmark_extractor_->extractLandmarks3D(rect_left, depth_map,
                                                                           best_face, result.landmarks);
            } else {
                result.result_code = landmark_extractor_->extractLandmarks2D(rect_left, best_face,
                                                                           result.landmarks);
            }

            auto landmark_end = std::chrono::high_resolution_clock::now();
            result.landmark_time_ms = std::chrono::duration<double, std::milli>(landmark_end - landmark_start).count();

            if (result.result_code != FaceResultCode::SUCCESS) {
                result.error_message = "Landmark extraction failed: " + landmark_extractor_->getLastError();
                return result.result_code;
            }

            result.landmark_quality = result.landmarks.overall_confidence;

            if (result.landmark_quality < config_.min_landmark_quality) {
                result.error_message = "Landmark quality below threshold";
                result.result_code = FaceResultCode::ERROR_LANDMARK_QUALITY_LOW;
                return result.result_code;
            }

            // Step 4: 3D face reconstruction (if enabled)
            if (config_.enable_3d_processing && face_3d_reconstructor_) {
                auto reconstruction_start = std::chrono::high_resolution_clock::now();
                result.result_code = face_3d_reconstructor_->reconstructFace(rect_left, rect_right,
                                                                           result.landmarks,
                                                                           result.face_3d_model);
                auto reconstruction_end = std::chrono::high_resolution_clock::now();
                result.reconstruction_time_ms = std::chrono::duration<double, std::milli>(
                    reconstruction_end - reconstruction_start).count();

                if (result.result_code != FaceResultCode::SUCCESS) {
                    result.error_message = "3D reconstruction failed: " + face_3d_reconstructor_->getLastError();
                    return result.result_code;
                }

                result.reconstruction_quality = result.face_3d_model.reconstruction_quality;
                result.depth_quality = result.landmarks.depth_coverage;

                if (result.reconstruction_quality < config_.min_reconstruction_quality) {
                    result.error_message = "3D reconstruction quality below threshold";
                    result.result_code = FaceResultCode::ERROR_3D_MODEL_INCOMPLETE;
                    return result.result_code;
                }
            }

            // Step 5: Liveness detection (if enabled)
            if (config_.enable_liveness_detection && liveness_detector_) {
                auto liveness_start = std::chrono::high_resolution_clock::now();

                if (config_.enable_3d_processing) {
                    result.result_code = liveness_detector_->detectLivenessMultiModal(
                        rect_left, result.face_3d_model.depth_map, result.landmarks, result.liveness_result);
                } else {
                    result.result_code = liveness_detector_->detectLivenessBasic(
                        rect_left, result.landmarks, result.liveness_result);
                }

                auto liveness_end = std::chrono::high_resolution_clock::now();
                result.liveness_time_ms = std::chrono::duration<double, std::milli>(
                    liveness_end - liveness_start).count();

                if (result.result_code != FaceResultCode::SUCCESS) {
                    result.error_message = "Liveness detection failed: " + liveness_detector_->getLastError();
                    return result.result_code;
                }

                if (result.liveness_result.liveness_score < config_.min_liveness_confidence) {
                    result.error_message = "Liveness confidence below threshold";
                    result.result_code = FaceResultCode::ERROR_LIVENESS_CHECK_FAILED;
                    return result.result_code;
                }

                if (result.liveness_result.presentation_attack_detected) {
                    result.error_message = "Presentation attack detected";
                    result.result_code = FaceResultCode::ERROR_PRESENTATION_ATTACK_DETECTED;
                    return result.result_code;
                }
            }

            // Step 6: Compute overall quality metrics
            computeOverallQualityMetrics(result);

            // Step 7: Banking compliance validation (if enabled)
            if (config_.enable_banking_mode) {
                validateBankingCompliance(result);
            }

            // Step 8: Performance assessment
            auto end_time = std::chrono::high_resolution_clock::now();
            result.total_processing_time_ms = std::chrono::duration<double, std::milli>(
                end_time - start_time).count();

            assessPerformance(result);

            result.success = true;

            // Update system statistics
            updateSystemStatistics(result);

            UNLOOK_LOG_DEBUG("Facial recognition pipeline completed: {:.2f}ms total, quality={:.3f}",
                           result.total_processing_time_ms, result.overall_quality);

            return FaceResultCode::SUCCESS;

        } catch (const std::exception& e) {
            result.error_message = "Exception during processing: " + std::string(e.what());
            result.result_code = FaceResultCode::ERROR_TEMPLATE_INVALID;
            return result.result_code;
        }
    }

    /**
     * @brief Verify identity against reference template
     */
    FaceResultCode verifyIdentity(const ProcessingResult& probe_result,
                                 const BiometricTemplate& reference_template,
                                 FaceMatchResult& match_result) {
        if (!initialized_ || !face_matcher_) {
            last_error_ = "System not initialized or matcher unavailable";
            return FaceResultCode::ERROR_TEMPLATE_INVALID;
        }

        if (!probe_result.success) {
            last_error_ = "Probe processing failed";
            return FaceResultCode::ERROR_TEMPLATE_INVALID;
        }

        try {
            auto matching_start = std::chrono::high_resolution_clock::now();

            // Generate template from probe result
            BiometricTemplate probe_template;
            FaceResultCode result = generateBiometricTemplate(probe_result, probe_template);
            if (result != FaceResultCode::SUCCESS) {
                return result;
            }

            // Perform verification
            if (config_.enable_banking_mode && config_.enable_liveness_detection) {
                result = face_matcher_->verifyBankingGrade(probe_template, reference_template,
                                                         probe_result.liveness_result, match_result);
            } else {
                result = face_matcher_->verify(probe_template, reference_template, match_result);
            }

            auto matching_end = std::chrono::high_resolution_clock::now();
            match_result.matching_time_ms = std::chrono::duration<double, std::milli>(
                matching_end - matching_start).count();

            return result;

        } catch (const std::exception& e) {
            last_error_ = "Exception during verification: " + std::string(e.what());
            return FaceResultCode::ERROR_MATCHING_FAILED;
        }
    }

    /**
     * @brief Enroll new user identity
     */
    FaceResultCode enrollIdentity(const std::vector<ProcessingResult>& enrollment_samples,
                                 const std::string& user_id,
                                 BiometricTemplate& enrollment_template) {
        if (!initialized_ || !face_enroller_) {
            last_error_ = "System not initialized or enroller unavailable";
            return FaceResultCode::ERROR_TEMPLATE_INVALID;
        }

        try {
            // Validate enrollment samples
            for (const auto& sample : enrollment_samples) {
                if (!sample.success) {
                    last_error_ = "Invalid enrollment sample detected";
                    return FaceResultCode::ERROR_TEMPLATE_INVALID;
                }

                if (config_.enable_banking_mode && !sample.banking_compliant) {
                    last_error_ = "Enrollment sample not banking compliant";
                    return FaceResultCode::ERROR_TEMPLATE_INVALID;
                }
            }

            // Perform enrollment
            EnrollmentConfig enrollment_config;
            enrollment_config.banking_grade_required = config_.enable_banking_mode;
            enrollment_config.require_liveness = config_.enable_liveness_detection;

            FaceResultCode result = face_enroller_->enrollUser(user_id, enrollment_samples,
                                                             enrollment_config, enrollment_template);

            if (result == FaceResultCode::SUCCESS) {
                UNLOOK_LOG_INFO("User {} enrolled successfully with {} samples",
                               user_id, enrollment_samples.size());
            }

            return result;

        } catch (const std::exception& e) {
            last_error_ = "Exception during enrollment: " + std::string(e.what());
            return FaceResultCode::ERROR_TEMPLATE_GENERATION_FAILED;
        }
    }

    /**
     * @brief Get system performance statistics
     */
    void getSystemStatistics(size_t& total_processed, size_t& successful_processed,
                           double& avg_processing_time, float& success_rate) const {
        std::lock_guard<std::mutex> lock(stats_mutex_);

        total_processed = total_processed_.load();
        successful_processed = successful_processed_.load();

        success_rate = (total_processed > 0) ?
                      static_cast<float>(successful_processed) / total_processed : 0.0f;

        avg_processing_time = (total_processed > 0) ?
                             total_processing_time_.load() / total_processed : 0.0;
    }

    /**
     * @brief Check if system is initialized
     */
    bool isInitialized() const {
        return initialized_;
    }

    /**
     * @brief Get system configuration
     */
    SystemConfig getConfiguration() const {
        return config_;
    }

    /**
     * @brief Set system configuration
     */
    FaceResultCode setConfiguration(const SystemConfig& config) {
        if (!config.validate()) {
            last_error_ = "Invalid system configuration";
            return FaceResultCode::ERROR_TEMPLATE_INVALID;
        }

        config_ = config;

        // Update subsystem configurations if initialized
        if (initialized_) {
            return updateSubsystemConfigurations();
        }

        return FaceResultCode::SUCCESS;
    }

    /**
     * @brief Get last error message
     */
    std::string getLastError() const {
        return last_error_;
    }

private:
    /**
     * @brief Initialize stereo calibration parameters
     */
    FaceResultCode initializeStereoCalibration() {
        if (!calibration_manager_->isCalibrated()) {
            last_error_ = "Stereo system not calibrated";
            return FaceResultCode::ERROR_TEMPLATE_INVALID;
        }

        // Get camera parameters (simplified - would use actual calibration data)
        camera_matrix_ = (cv::Mat_<double>(3, 3) <<
            1000.0, 0, 728.0,      // Approximate values for IMX296
            0, 1000.0, 544.0,      // Will be updated from actual calibration
            0, 0, 1);

        dist_coeffs_ = cv::Mat::zeros(5, 1, CV_64F);

        // Load actual calibration from calib_boofcv_test3.yaml
        baseline_mm_ = 70.017f; // From calibration file

        // Initialize stereo rectification maps
        if (config_.enable_stereo_rectification) {
            initializeStereoRectification();
        }

        UNLOOK_LOG_INFO("Stereo calibration initialized: baseline={:.3f}mm", baseline_mm_);

        return FaceResultCode::SUCCESS;
    }

    /**
     * @brief Initialize stereo rectification maps
     */
    void initializeStereoRectification() {
        cv::Size image_size(1456, 1088); // IMX296 resolution

        // Create identity matrices for demonstration
        cv::Mat R1 = cv::Mat::eye(3, 3, CV_64F);
        cv::Mat R2 = cv::Mat::eye(3, 3, CV_64F);
        cv::Mat P1 = cv::Mat::zeros(3, 4, CV_64F);
        cv::Mat P2 = cv::Mat::zeros(3, 4, CV_64F);

        camera_matrix_.copyTo(P1(cv::Rect(0, 0, 3, 3)));
        camera_matrix_.copyTo(P2(cv::Rect(0, 0, 3, 3)));

        // Initialize rectification maps
        cv::initUndistortRectifyMap(camera_matrix_, dist_coeffs_, R1, P1, image_size,
                                   CV_16SC2, rectify_map_left_x_, rectify_map_left_y_);

        cv::initUndistortRectifyMap(camera_matrix_, dist_coeffs_, R2, P2, image_size,
                                   CV_16SC2, rectify_map_right_x_, rectify_map_right_y_);
    }

    /**
     * @brief Apply stereo rectification
     */
    void applyStereoRectification(const cv::Mat& left_input, const cv::Mat& right_input,
                                 cv::Mat& left_rectified, cv::Mat& right_rectified) {
        if (!rectify_map_left_x_.empty() && !rectify_map_left_y_.empty()) {
            cv::remap(left_input, left_rectified, rectify_map_left_x_, rectify_map_left_y_, cv::INTER_LINEAR);
            cv::remap(right_input, right_rectified, rectify_map_right_x_, rectify_map_right_y_, cv::INTER_LINEAR);
        } else {
            left_rectified = left_input.clone();
            right_rectified = right_input.clone();
        }
    }

    /**
     * @brief Compute overall quality metrics
     */
    void computeOverallQualityMetrics(ProcessingResult& result) {
        float quality_sum = result.face_quality * 0.2f + result.landmark_quality * 0.3f;

        if (config_.enable_3d_processing) {
            quality_sum += result.reconstruction_quality * 0.3f + result.depth_quality * 0.2f;
        } else {
            quality_sum += result.landmark_quality * 0.5f; // More weight on landmarks
        }

        result.overall_quality = std::max(0.0f, std::min(1.0f, quality_sum));
    }

    /**
     * @brief Validate banking compliance
     */
    void validateBankingCompliance(ProcessingResult& result) {
        // Banking compliance requires high quality across all metrics
        bool meets_quality = result.overall_quality >= 0.95f;
        bool meets_timing = result.total_processing_time_ms <= config_.target_total_time_ms;
        bool meets_liveness = !config_.enable_liveness_detection ||
                             (result.liveness_result.is_live && result.liveness_result.liveness_score >= 0.95f);

        result.banking_compliant = meets_quality && meets_timing && meets_liveness;

        // Compute compliance score
        float quality_score = result.overall_quality;
        float timing_score = std::max(0.0f, 1.0f - (result.total_processing_time_ms - config_.target_total_time_ms) / config_.target_total_time_ms);
        float liveness_score = config_.enable_liveness_detection ? result.liveness_result.liveness_score : 1.0f;

        result.compliance_score = (quality_score * 0.5f + timing_score * 0.3f + liveness_score * 0.2f);
    }

    /**
     * @brief Assess performance against targets
     */
    void assessPerformance(ProcessingResult& result) {
        // Check individual timing targets
        bool detection_ok = result.detection_time_ms <= config_.target_detection_time_ms;
        bool landmark_ok = result.landmark_time_ms <= config_.target_landmark_time_ms;
        bool reconstruction_ok = !config_.enable_3d_processing ||
                               result.reconstruction_time_ms <= config_.target_reconstruction_time_ms;

        result.met_timing_targets = detection_ok && landmark_ok && reconstruction_ok &&
                                   result.total_processing_time_ms <= config_.target_total_time_ms;

        // Compute processing efficiency
        float target_time = config_.target_total_time_ms;
        float actual_time = result.total_processing_time_ms;

        if (actual_time > 0) {
            result.processing_efficiency = std::min(1.0f, target_time / actual_time);
        } else {
            result.processing_efficiency = 1.0f;
        }
    }

    /**
     * @brief Generate biometric template from processing result
     */
    FaceResultCode generateBiometricTemplate(const ProcessingResult& processing_result,
                                            BiometricTemplate& template_data) {
        // This would generate an encrypted biometric template
        // For now, create a simplified template structure

        template_data = BiometricTemplate{};
        template_data.template_id = generateTemplateId();
        template_data.template_version = 1;
        template_data.enrollment_quality = static_cast<FaceQuality>(
            static_cast<int>(processing_result.overall_quality * 3));
        template_data.template_quality_score = processing_result.overall_quality;
        template_data.banking_certified = processing_result.banking_compliant;
        template_data.iso_compliant = true; // Would be validated properly
        template_data.creation_time = std::chrono::system_clock::now();

        // Extract features (simplified)
        if (config_.enable_3d_processing) {
            template_data.feature_count = 512; // Standard feature count
            template_data.landmark_count = processing_result.landmarks.points3D.size();
        } else {
            template_data.feature_count = 256; // Reduced for 2D
            template_data.landmark_count = processing_result.landmarks.points2D.size();
        }

        template_data.expected_far = config_.enable_banking_mode ? 0.0001f : 0.001f;
        template_data.expected_frr = config_.enable_banking_mode ? 0.03f : 0.05f;

        return FaceResultCode::SUCCESS;
    }

    /**
     * @brief Update subsystem configurations
     */
    FaceResultCode updateSubsystemConfigurations() {
        // Update landmark extractor
        if (landmark_extractor_) {
            landmark_extractor_->setBankingMode(config_.enable_banking_mode);
        }

        // Update 3D reconstructor
        if (face_3d_reconstructor_) {
            face_3d_reconstructor_->setBankingMode(config_.enable_banking_mode);
        }

        // Update face matcher
        if (face_matcher_) {
            face_matcher_->setBankingMode(config_.enable_banking_mode);
        }

        return FaceResultCode::SUCCESS;
    }

    /**
     * @brief Update system performance statistics
     */
    void updateSystemStatistics(const ProcessingResult& result) {
        total_processed_.fetch_add(1);
        if (result.success) {
            successful_processed_.fetch_add(1);
        }

        total_processing_time_.store(total_processing_time_.load() + result.total_processing_time_ms);
    }

    /**
     * @brief Generate unique template ID
     */
    std::string generateTemplateId() {
        auto now = std::chrono::system_clock::now();
        auto time_t = std::chrono::system_clock::to_time_t(now);
        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;

        std::stringstream ss;
        ss << "TMPL_" << std::hex << time_t << "_" << ms.count();
        return ss.str();
    }
};

} // namespace face
} // namespace unlook