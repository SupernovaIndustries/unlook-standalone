#pragma once

#include "unlook/face/FaceTypes.hpp"
#include "unlook/face/FaceDetector.hpp"
#include "unlook/face/LandmarkExtractor.hpp"
#include "unlook/face/Face3DReconstructor.hpp"
#include "unlook/face/FaceEncoder.hpp"
#include "unlook/face/LivenessDetector.hpp"
#include <opencv2/opencv.hpp>
#include <memory>
#include <vector>
#include <functional>
#include <mutex>
#include <atomic>
#include <thread>
#include <condition_variable>

namespace unlook {
namespace face {

/**
 * @brief Face enrollment session data
 */
struct EnrollmentSession {
    // Session identification
    std::string session_id;               ///< Unique session identifier
    std::string user_id;                  ///< User being enrolled
    std::chrono::system_clock::time_point start_time; ///< Session start time

    // Collected samples
    std::vector<cv::Mat> face_images;     ///< Collected face images
    std::vector<cv::Mat> depth_maps;      ///< Corresponding depth maps
    std::vector<FacialLandmarks> landmarks_sequence; ///< Landmarks per sample
    std::vector<Face3DModel> face_models; ///< 3D models per sample
    std::vector<LivenessResult> liveness_results; ///< Liveness per sample

    // Quality metrics
    std::vector<float> sample_qualities;  ///< Quality score per sample
    float overall_quality;                ///< Overall enrollment quality
    float pose_coverage;                  ///< Pose variation coverage (0-1)
    float lighting_coverage;              ///< Lighting variation coverage (0-1)

    // Progress tracking
    EnrollmentState current_state;        ///< Current enrollment state
    int samples_captured;                 ///< Number of samples captured
    int samples_required;                 ///< Total samples required
    float completion_percentage;          ///< Overall completion (0-100)

    // User guidance
    std::string current_instruction;      ///< Current user instruction
    cv::Point2f target_face_center;       ///< Target face center position
    cv::Size2f target_face_size;          ///< Target face size
    bool ready_to_capture;                ///< Ready for next capture

    // Error tracking
    FaceResultCode last_error;            ///< Last error encountered
    std::string error_message;            ///< Human-readable error
    int consecutive_failures;             ///< Consecutive failure count

    EnrollmentSession() : overall_quality(0), pose_coverage(0), lighting_coverage(0),
                         current_state(EnrollmentState::NOT_STARTED), samples_captured(0),
                         samples_required(3), completion_percentage(0), ready_to_capture(false),
                         last_error(FaceResultCode::SUCCESS), consecutive_failures(0) {}

    bool isComplete() const {
        return current_state == EnrollmentState::COMPLETED;
    }

    bool hasFailed() const {
        return current_state == EnrollmentState::FAILED;
    }

    int getRemainingSamples() const {
        return std::max(0, samples_required - samples_captured);
    }
};

/**
 * @brief Intel RealSense style face enrollment system
 *
 * Professional face enrollment system that guides users through a
 * comprehensive enrollment process similar to Intel RealSense ID.
 * Collects multiple high-quality face samples with pose and lighting
 * variation for robust template generation.
 *
 * Key features:
 * - Interactive user guidance with real-time feedback
 * - Multi-sample collection with pose variation requirements
 * - Integrated liveness detection during enrollment
 * - Quality-gated sample acceptance
 * - 3D face model generation and validation
 * - Banking-grade template generation
 * - Progress tracking and error recovery
 * - Real-time preview with overlay guidance
 * - ARM64/CM4 optimized for smooth operation
 *
 * Provides Intel RealSense style user experience with banking security.
 */
class FaceEnroller {
public:
    /**
     * @brief Constructor with default components
     */
    FaceEnroller();

    /**
     * @brief Constructor with custom components
     * @param detector Face detector component
     * @param landmark_extractor Landmark extractor component
     * @param reconstructor 3D face reconstructor component
     * @param encoder Face encoder component
     * @param liveness_detector Liveness detector component
     */
    FaceEnroller(std::shared_ptr<FaceDetector> detector,
                std::shared_ptr<LandmarkExtractor> landmark_extractor,
                std::shared_ptr<Face3DReconstructor> reconstructor,
                std::shared_ptr<FaceEncoder> encoder,
                std::shared_ptr<LivenessDetector> liveness_detector);

    /**
     * @brief Destructor
     */
    ~FaceEnroller();

    /**
     * @brief Initialize enrollment system
     * @param config Enrollment configuration
     * @return FaceResultCode indicating success or failure
     */
    FaceResultCode initialize(const EnrollmentConfig& config);

    /**
     * @brief Check if enroller is initialized
     * @return True if enroller is ready for use
     */
    bool isInitialized() const;

    /**
     * @brief Start new enrollment session
     * @param user_id User identifier for enrollment
     * @param session Optionally provide existing session to resume
     * @return FaceResultCode indicating success or failure
     */
    FaceResultCode startEnrollment(const std::string& user_id,
                                  EnrollmentSession* session = nullptr);

    /**
     * @brief Process frame during enrollment
     * @param rgb_image Input RGB image (CV_8UC3)
     * @param depth_map Input depth map in millimeters (CV_32F)
     * @param progress Output enrollment progress
     * @return FaceResultCode indicating processing result
     */
    FaceResultCode processEnrollmentFrame(const cv::Mat& rgb_image,
                                         const cv::Mat& depth_map,
                                         EnrollmentProgress& progress);

    /**
     * @brief Manually capture current frame as enrollment sample
     * @param force_capture Force capture even if quality is low
     * @return FaceResultCode indicating capture result
     */
    FaceResultCode captureEnrollmentSample(bool force_capture = false);

    /**
     * @brief Complete enrollment and generate final template
     * @param final_template Output generated biometric template
     * @return FaceResultCode indicating completion result
     */
    FaceResultCode completeEnrollment(BiometricTemplate& final_template);

    /**
     * @brief Cancel current enrollment session
     * @return FaceResultCode indicating cancellation result
     */
    FaceResultCode cancelEnrollment();

    /**
     * @brief Get current enrollment progress
     * @param progress Output progress information
     * @return FaceResultCode indicating success or failure
     */
    FaceResultCode getEnrollmentProgress(EnrollmentProgress& progress) const;

    /**
     * @brief Get current enrollment session data
     * @param session Output session data
     * @return FaceResultCode indicating success or failure
     */
    FaceResultCode getEnrollmentSession(EnrollmentSession& session) const;

    /**
     * @brief Generate preview image with enrollment guidance overlay
     * @param input_image Input RGB image
     * @param preview_image Output image with guidance overlay
     * @param show_landmarks Show detected landmarks
     * @param show_quality_metrics Show quality information
     * @return FaceResultCode indicating success or failure
     */
    FaceResultCode generateEnrollmentPreview(const cv::Mat& input_image,
                                            cv::Mat& preview_image,
                                            bool show_landmarks = true,
                                            bool show_quality_metrics = true);

    /**
     * @brief Set enrollment progress callback
     * @param callback Callback function for progress updates
     */
    void setProgressCallback(EnrollmentProgressCallback callback);

    /**
     * @brief Set real-time frame processing callback
     * @param callback Callback for each processed frame
     */
    void setFrameProcessingCallback(
        std::function<void(const cv::Mat&, const EnrollmentProgress&)> callback);

    /**
     * @brief Set enrollment configuration
     * @param config New enrollment configuration
     * @return FaceResultCode indicating success or failure
     */
    FaceResultCode setConfiguration(const EnrollmentConfig& config);

    /**
     * @brief Get current enrollment configuration
     * @return Current enrollment configuration
     */
    EnrollmentConfig getConfiguration() const;

    /**
     * @brief Save enrollment session to file
     * @param session_path Path to save session file
     * @param include_images Include images in saved session
     * @return FaceResultCode indicating success or failure
     */
    FaceResultCode saveEnrollmentSession(const std::string& session_path,
                                        bool include_images = false);

    /**
     * @brief Load enrollment session from file
     * @param session_path Path to session file
     * @param session Output loaded session
     * @return FaceResultCode indicating success or failure
     */
    FaceResultCode loadEnrollmentSession(const std::string& session_path,
                                        EnrollmentSession& session);

    /**
     * @brief Resume enrollment from saved session
     * @param session Previously saved session
     * @return FaceResultCode indicating success or failure
     */
    FaceResultCode resumeEnrollment(const EnrollmentSession& session);

    /**
     * @brief Validate enrollment sample quality
     * @param rgb_image Sample RGB image
     * @param depth_map Sample depth map
     * @param landmarks Sample landmarks
     * @param quality_score Output quality score (0-1)
     * @param quality_issues Output list of quality issues
     * @return True if sample meets quality requirements
     */
    bool validateSampleQuality(const cv::Mat& rgb_image,
                              const cv::Mat& depth_map,
                              const FacialLandmarks& landmarks,
                              float& quality_score,
                              std::vector<std::string>& quality_issues);

    /**
     * @brief Remove poor quality samples from session
     * @param min_quality_threshold Minimum quality to keep
     * @return Number of samples removed
     */
    int removePoorQualitySamples(float min_quality_threshold = 0.7f);

    /**
     * @brief Get enrollment statistics
     * @param total_sessions Total enrollment sessions started
     * @param completed_sessions Successfully completed sessions
     * @param average_duration Average enrollment duration (seconds)
     * @param success_rate Overall enrollment success rate
     */
    void getEnrollmentStatistics(size_t& total_sessions,
                                size_t& completed_sessions,
                                double& average_duration,
                                float& success_rate) const;

    /**
     * @brief Reset enrollment statistics
     */
    void resetStatistics();

    /**
     * @brief Get last error message
     * @return Human-readable error message
     */
    std::string getLastError() const;

    /**
     * @brief Check if enrollment is currently active
     * @return True if enrollment session is in progress
     */
    bool isEnrollmentActive() const;

    /**
     * @brief Get required pose variations for complete enrollment
     * @return Vector of required pose descriptions
     */
    static std::vector<std::string> getRequiredPoseVariations();

    /**
     * @brief Get recommended enrollment environment setup
     * @return Formatted string with setup recommendations
     */
    static std::string getEnvironmentRecommendations();

    /**
     * @brief Enable banking-grade enrollment mode
     * @param enable Enable/disable banking mode
     * @param require_liveness Require liveness for all samples
     * @param min_samples Minimum samples for banking grade
     */
    void setBankingMode(bool enable,
                       bool require_liveness = true,
                       int min_samples = 5);

private:
    // Core components
    std::shared_ptr<FaceDetector> face_detector_;
    std::shared_ptr<LandmarkExtractor> landmark_extractor_;
    std::shared_ptr<Face3DReconstructor> face_reconstructor_;
    std::shared_ptr<FaceEncoder> face_encoder_;
    std::shared_ptr<LivenessDetector> liveness_detector_;

    // Configuration and state
    EnrollmentConfig config_;
    std::unique_ptr<EnrollmentSession> current_session_;
    mutable std::mutex session_mutex_;

    // Current frame analysis state
    std::mutex frame_mutex_;
    cv::Mat current_rgb_frame_;
    cv::Mat current_depth_frame_;
    FaceDetectionResult current_detection_;
    FacialLandmarks current_landmarks_;
    Face3DModel current_3d_model_;
    LivenessResult current_liveness_;
    bool has_current_frame_{false};

    // Callbacks
    std::mutex callback_mutex_;
    EnrollmentProgressCallback progress_callback_;
    std::function<void(const cv::Mat&, const EnrollmentProgress&)> frame_callback_;

    // Performance monitoring
    mutable std::mutex stats_mutex_;
    std::atomic<size_t> total_sessions_{0};
    std::atomic<size_t> completed_sessions_{0};
    std::atomic<double> total_enrollment_time_{0.0};

    /**
     * @brief Update enrollment state based on current analysis
     * @param detection Current face detection
     * @param landmarks Current landmarks
     * @param liveness Current liveness result
     */
    void updateEnrollmentState(const FaceDetectionResult& detection,
                              const FacialLandmarks& landmarks,
                              const LivenessResult& liveness);

    /**
     * @brief Generate user guidance instruction
     * @param detection Current face detection
     * @param landmarks Current landmarks
     * @param quality_score Current sample quality
     * @return User instruction string
     */
    std::string generateUserInstruction(const FaceDetectionResult& detection,
                                       const FacialLandmarks& landmarks,
                                       float quality_score);

    /**
     * @brief Check if current sample should be automatically captured
     * @param quality_score Current quality score
     * @param pose_variation Pose variation from previous samples
     * @return True if sample should be captured
     */
    bool shouldAutoCapture(float quality_score, float pose_variation);

    /**
     * @brief Compute pose variation from existing samples
     * @param new_landmarks New landmark set
     * @return Pose variation score (0-1)
     */
    float computePoseVariation(const FacialLandmarks& new_landmarks);

    /**
     * @brief Compute lighting variation from existing samples
     * @param new_image New face image
     * @return Lighting variation score (0-1)
     */
    float computeLightingVariation(const cv::Mat& new_image);

    /**
     * @brief Assess overall enrollment quality
     * @return Overall quality score (0-1)
     */
    float assessOverallQuality();

    /**
     * @brief Generate enrollment guidance overlay
     * @param input_image Input image
     * @param output_image Output image with overlay
     * @param detection Current detection
     * @param progress Current progress
     */
    void generateGuidanceOverlay(const cv::Mat& input_image,
                                cv::Mat& output_image,
                                const FaceDetectionResult& detection,
                                const EnrollmentProgress& progress);

    /**
     * @brief Draw face guidance box and target
     * @param image Input/output image
     * @param target_center Target face center
     * @param target_size Target face size
     * @param current_box Current face detection box
     * @param ready_color Color for guidance elements
     */
    void drawFaceGuidance(cv::Mat& image,
                         const cv::Point2f& target_center,
                         const cv::Size2f& target_size,
                         const cv::Rect2f& current_box,
                         const cv::Scalar& ready_color);

    /**
     * @brief Draw enrollment progress indicators
     * @param image Input/output image
     * @param progress Current enrollment progress
     */
    void drawProgressIndicators(cv::Mat& image,
                               const EnrollmentProgress& progress);

    /**
     * @brief Draw quality metrics overlay
     * @param image Input/output image
     * @param quality_score Overall quality score
     * @param liveness_result Liveness detection result
     * @param pose_variation Pose variation score
     */
    void drawQualityMetrics(cv::Mat& image,
                           float quality_score,
                           const LivenessResult& liveness_result,
                           float pose_variation);

    /**
     * @brief Update progress callback if set
     * @param progress Current progress
     */
    void notifyProgressCallback(const EnrollmentProgress& progress);

    /**
     * @brief Update frame processing callback if set
     * @param frame Current frame
     * @param progress Current progress
     */
    void notifyFrameCallback(const cv::Mat& frame,
                            const EnrollmentProgress& progress);

    /**
     * @brief Update enrollment statistics
     * @param session_completed Whether session was completed successfully
     * @param duration_seconds Session duration in seconds
     */
    void updateStatistics(bool session_completed, double duration_seconds);

    /**
     * @brief Validate enrollment completion requirements
     * @return True if enrollment can be completed
     */
    bool canCompleteEnrollment() const;

    /**
     * @brief Generate error recovery suggestions
     * @param error_code Current error code
     * @param consecutive_failures Number of consecutive failures
     * @return Recovery suggestion string
     */
    std::string generateRecoverySuggestion(FaceResultCode error_code,
                                          int consecutive_failures);

    // Disable copy constructor and assignment
    FaceEnroller(const FaceEnroller&) = delete;
    FaceEnroller& operator=(const FaceEnroller&) = delete;

    // Enable move semantics
    FaceEnroller(FaceEnroller&&) noexcept;
    FaceEnroller& operator=(FaceEnroller&&) noexcept;
};

/**
 * @brief Enrollment guidance and user experience utilities
 */
class EnrollmentGuidance {
public:
    /**
     * @brief Generate optimal face positioning target
     * @param image_size Input image dimensions
     * @param target_center Output target face center
     * @param target_size Output target face size
     * @param margin_ratio Safety margin ratio
     */
    static void generateOptimalTarget(const cv::Size& image_size,
                                     cv::Point2f& target_center,
                                     cv::Size2f& target_size,
                                     float margin_ratio = 0.1f);

    /**
     * @brief Compute face positioning score
     * @param current_box Current face detection
     * @param target_center Target face center
     * @param target_size Target face size
     * @return Positioning score (0-1)
     */
    static float computePositioningScore(const cv::Rect2f& current_box,
                                        const cv::Point2f& target_center,
                                        const cv::Size2f& target_size);

    /**
     * @brief Generate user instruction based on face position
     * @param positioning_score Current positioning score
     * @param face_box Current face detection
     * @param target_center Target center
     * @param target_size Target size
     * @return User instruction string
     */
    static std::string generatePositionInstruction(float positioning_score,
                                                   const cv::Rect2f& face_box,
                                                   const cv::Point2f& target_center,
                                                   const cv::Size2f& target_size);

    /**
     * @brief Get enrollment pose sequence recommendations
     * @param num_samples Number of samples to collect
     * @return Vector of pose descriptions in recommended order
     */
    static std::vector<std::string> getPoseSequence(int num_samples);

    /**
     * @brief Compute enrollment completion confidence
     * @param session Enrollment session data
     * @return Completion confidence score (0-1)
     */
    static float computeCompletionConfidence(const EnrollmentSession& session);
};

} // namespace face
} // namespace unlook