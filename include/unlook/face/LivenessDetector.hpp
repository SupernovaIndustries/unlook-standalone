#pragma once

#include "unlook/face/FaceTypes.hpp"
#include "unlook/stereo/DepthProcessor.hpp"
#include <opencv2/opencv.hpp>
#include <memory>
#include <vector>
#include <queue>
#include <mutex>
#include <atomic>
#include <chrono>

namespace unlook {
namespace face {

/**
 * @brief Liveness detection configuration
 */
struct LivenessDetectionConfig {
    // Detection methods to enable
    bool enable_depth_liveness = true;        ///< Enable depth-based liveness
    bool enable_motion_liveness = true;       ///< Enable motion-based liveness
    bool enable_texture_liveness = true;      ///< Enable texture analysis liveness
    bool enable_pulse_detection = false;      ///< Enable pulse/blood flow detection

    // Depth-based liveness parameters
    float min_depth_variation_mm = 2.0f;      ///< Minimum depth variation for liveness
    float max_depth_noise_mm = 1.0f;          ///< Maximum acceptable depth noise
    float depth_confidence_threshold = 0.8f;  ///< Minimum depth confidence
    bool require_3d_face_structure = true;    ///< Require valid 3D face structure

    // Motion-based liveness parameters
    float min_motion_magnitude = 1.0f;        ///< Minimum motion for liveness (pixels)
    float max_motion_magnitude = 20.0f;       ///< Maximum acceptable motion
    int motion_analysis_frames = 10;          ///< Number of frames for motion analysis
    bool require_natural_motion = true;       ///< Require natural motion patterns

    // Texture analysis parameters
    float texture_quality_threshold = 0.7f;   ///< Minimum texture quality
    bool enable_frequency_analysis = true;    ///< Enable frequency domain analysis
    bool enable_lbp_analysis = true;          ///< Enable Local Binary Patterns
    float print_attack_threshold = 0.6f;      ///< Photo/print attack detection threshold

    // Temporal analysis parameters
    int temporal_window_frames = 30;          ///< Temporal analysis window size
    float temporal_consistency_threshold = 0.8f; ///< Temporal consistency requirement
    bool enable_blink_detection = true;       ///< Enable eye blink detection
    float min_blink_frequency = 0.1f;         ///< Minimum blink frequency (Hz)
    float max_blink_frequency = 1.0f;         ///< Maximum blink frequency (Hz)

    // Challenge-response parameters
    bool enable_challenge_response = false;   ///< Enable active challenges
    std::vector<std::string> challenge_types; ///< Types of challenges ("nod", "smile", "turn")
    float challenge_timeout_seconds = 10.0f;  ///< Timeout for challenges

    // Quality gates
    float min_face_size_ratio = 0.15f;        ///< Minimum face size (fraction of image)
    float max_face_size_ratio = 0.8f;         ///< Maximum face size
    float min_face_quality = 0.7f;           ///< Minimum face detection quality
    bool require_frontal_pose = true;        ///< Require near-frontal face pose

    // Banking-grade requirements
    bool banking_grade_mode = false;          ///< Enable banking-grade detection
    float banking_confidence_threshold = 0.95f; ///< Banking grade confidence
    bool require_multi_modal = true;         ///< Require multiple detection methods
    int min_analysis_frames = 15;            ///< Minimum frames for analysis

    // Performance settings
    bool enable_gpu_acceleration = false;     ///< Use GPU acceleration
    bool enable_multithreading = true;       ///< Multi-threaded processing
    int max_processing_threads = 2;          ///< Maximum processing threads

    bool validate() const;
    std::string toString() const;
};

/**
 * @brief Multi-modal liveness detection for anti-spoofing
 *
 * Advanced liveness detection system that combines multiple modalities
 * to detect presentation attacks (photos, videos, masks, etc.) with
 * banking-grade reliability and precision.
 *
 * Key features:
 * - Depth-based liveness using stereo vision data
 * - Motion analysis with natural movement patterns
 * - Texture analysis for print/display attack detection
 * - Temporal consistency analysis across frames
 * - Eye blink and micro-expression detection
 * - Challenge-response interactive liveness
 * - Banking-grade multi-modal fusion
 * - Real-time processing optimized for ARM64/CM4
 *
 * Integrates seamlessly with Unlook stereo depth system.
 */
class LivenessDetector {
public:
    /**
     * @brief Constructor with default configuration
     */
    LivenessDetector();

    /**
     * @brief Constructor with custom configuration
     * @param config Liveness detection configuration
     */
    explicit LivenessDetector(const LivenessDetectionConfig& config);

    /**
     * @brief Destructor
     */
    ~LivenessDetector();

    /**
     * @brief Initialize detector with depth processor
     * @param depth_processor Stereo depth processor for depth liveness
     * @return FaceResultCode indicating success or failure
     */
    FaceResultCode initialize(std::shared_ptr<stereo::DepthProcessor> depth_processor);

    /**
     * @brief Check if detector is initialized
     * @return True if detector is ready for use
     */
    bool isInitialized() const;

    /**
     * @brief Perform passive liveness detection on single frame
     * @param rgb_image Input RGB image (CV_8UC3)
     * @param depth_map Input depth map in millimeters (CV_32F)
     * @param face_box Face bounding box
     * @param landmarks Optional facial landmarks
     * @param liveness_result Output liveness detection result
     * @return FaceResultCode indicating success or failure
     */
    FaceResultCode detectLiveness(const cv::Mat& rgb_image,
                                 const cv::Mat& depth_map,
                                 const FaceBoundingBox& face_box,
                                 const FacialLandmarks* landmarks,
                                 LivenessResult& liveness_result);

    /**
     * @brief Perform temporal liveness analysis on frame sequence
     * @param rgb_frames Vector of RGB frames
     * @param depth_frames Vector of corresponding depth maps
     * @param face_boxes Vector of face detections per frame
     * @param landmarks_sequence Optional landmarks per frame
     * @param liveness_result Output temporal liveness result
     * @return FaceResultCode indicating success or failure
     */
    FaceResultCode detectTemporalLiveness(
        const std::vector<cv::Mat>& rgb_frames,
        const std::vector<cv::Mat>& depth_frames,
        const std::vector<FaceBoundingBox>& face_boxes,
        const std::vector<FacialLandmarks>* landmarks_sequence,
        LivenessResult& liveness_result);

    /**
     * @brief Start real-time liveness monitoring
     * @param callback Callback for liveness results
     * @return FaceResultCode indicating success or failure
     */
    FaceResultCode startRealtimeMonitoring(LivenessDetectionCallback callback);

    /**
     * @brief Process frame in real-time monitoring mode
     * @param rgb_image Input RGB frame
     * @param depth_map Input depth map
     * @param face_box Current face detection
     * @param landmarks Optional facial landmarks
     * @return FaceResultCode indicating success or failure
     */
    FaceResultCode processRealtimeFrame(const cv::Mat& rgb_image,
                                       const cv::Mat& depth_map,
                                       const FaceBoundingBox& face_box,
                                       const FacialLandmarks* landmarks = nullptr);

    /**
     * @brief Stop real-time liveness monitoring
     * @return FaceResultCode indicating success or failure
     */
    FaceResultCode stopRealtimeMonitoring();

    /**
     * @brief Perform challenge-response liveness test
     * @param challenge_type Type of challenge ("nod", "smile", "turn", "blink")
     * @param timeout_seconds Maximum time for user response
     * @param callback Callback for challenge result
     * @return FaceResultCode indicating success or failure
     */
    FaceResultCode performChallenge(const std::string& challenge_type,
                                   float timeout_seconds,
                                   LivenessDetectionCallback callback);

    /**
     * @brief Detect specific presentation attacks
     * @param rgb_image Input RGB image
     * @param depth_map Input depth map
     * @param face_box Face bounding box
     * @param attack_types Vector of attack types to check
     * @param attack_scores Output attack confidence scores
     * @return FaceResultCode indicating success or failure
     */
    FaceResultCode detectPresentationAttacks(
        const cv::Mat& rgb_image,
        const cv::Mat& depth_map,
        const FaceBoundingBox& face_box,
        const std::vector<std::string>& attack_types,
        std::map<std::string, float>& attack_scores);

    /**
     * @brief Banking-grade liveness verification
     * @param rgb_frames Multiple RGB frames for analysis
     * @param depth_frames Corresponding depth maps
     * @param face_boxes Face detections per frame
     * @param landmarks_sequence Landmarks per frame
     * @param banking_result Output banking-grade liveness result
     * @return FaceResultCode indicating success or failure
     */
    FaceResultCode verifyBankingGradeLiveness(
        const std::vector<cv::Mat>& rgb_frames,
        const std::vector<cv::Mat>& depth_frames,
        const std::vector<FaceBoundingBox>& face_boxes,
        const std::vector<FacialLandmarks>& landmarks_sequence,
        LivenessResult& banking_result);

    /**
     * @brief Set detection configuration
     * @param config New configuration
     * @return FaceResultCode indicating success or failure
     */
    FaceResultCode setConfiguration(const LivenessDetectionConfig& config);

    /**
     * @brief Get current configuration
     * @return Current detection configuration
     */
    LivenessDetectionConfig getConfiguration() const;

    /**
     * @brief Set depth processor for depth-based liveness
     * @param depth_processor Shared pointer to depth processor
     */
    void setDepthProcessor(std::shared_ptr<stereo::DepthProcessor> depth_processor);

    /**
     * @brief Enable/disable specific liveness methods
     * @param enable_depth Enable depth-based liveness
     * @param enable_motion Enable motion-based liveness
     * @param enable_texture Enable texture analysis
     * @param enable_temporal Enable temporal analysis
     */
    void setMethods(bool enable_depth,
                   bool enable_motion,
                   bool enable_texture,
                   bool enable_temporal);

    /**
     * @brief Set custom thresholds for liveness decisions
     * @param depth_threshold Depth liveness threshold
     * @param motion_threshold Motion liveness threshold
     * @param texture_threshold Texture liveness threshold
     * @param overall_threshold Overall liveness threshold
     */
    void setThresholds(float depth_threshold,
                      float motion_threshold,
                      float texture_threshold,
                      float overall_threshold);

    /**
     * @brief Get liveness detection statistics
     * @param avg_processing_time Average processing time per frame
     * @param total_detections Total number of liveness checks
     * @param live_detections Number of live detections
     * @param attack_detections Number of attack detections
     * @param current_accuracy Current detection accuracy
     */
    void getStatistics(double& avg_processing_time,
                      size_t& total_detections,
                      size_t& live_detections,
                      size_t& attack_detections,
                      float& current_accuracy) const;

    /**
     * @brief Reset detection statistics
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
     * @brief Get supported challenge types
     * @return Vector of supported challenge type names
     */
    static std::vector<std::string> getSupportedChallenges();

    /**
     * @brief Get supported presentation attack types
     * @return Vector of detectable presentation attack types
     */
    static std::vector<std::string> getSupportedAttackTypes();

    /**
     * @brief Validate liveness detection model
     * @param model_path Path to liveness model
     * @return True if model is valid and compatible
     */
    static bool validateModel(const std::string& model_path);

    /**
     * @brief Enable banking-grade liveness mode
     * @param enable Enable/disable banking mode
     * @param multi_modal_required Require multiple detection methods
     * @param strict_thresholds Use strict banking thresholds
     */
    void setBankingMode(bool enable,
                       bool multi_modal_required = true,
                       bool strict_thresholds = true);

private:
    class Impl;
    std::unique_ptr<Impl> pImpl;

    // Integration with depth processor
    std::shared_ptr<stereo::DepthProcessor> depth_processor_;

    // Frame buffer for temporal analysis
    std::mutex buffer_mutex_;
    std::queue<cv::Mat> rgb_frame_buffer_;
    std::queue<cv::Mat> depth_frame_buffer_;
    std::queue<FaceBoundingBox> face_buffer_;
    std::queue<std::chrono::system_clock::time_point> timestamp_buffer_;
    size_t max_buffer_size_{30};

    // Real-time monitoring state
    std::atomic<bool> realtime_active_{false};
    std::mutex callback_mutex_;
    LivenessDetectionCallback realtime_callback_;

    // Performance monitoring
    mutable std::mutex stats_mutex_;
    std::atomic<size_t> total_detections_{0};
    std::atomic<size_t> live_detections_{0};
    std::atomic<size_t> attack_detections_{0};
    std::atomic<double> total_processing_time_{0.0};

    /**
     * @brief Internal liveness detection implementation
     * @param rgb_image Input RGB image
     * @param depth_map Input depth map
     * @param face_box Face bounding box
     * @param landmarks Optional landmarks
     * @param use_temporal Use temporal information
     * @param liveness_result Output result
     * @return FaceResultCode
     */
    FaceResultCode detectLivenessInternal(const cv::Mat& rgb_image,
                                         const cv::Mat& depth_map,
                                         const FaceBoundingBox& face_box,
                                         const FacialLandmarks* landmarks,
                                         bool use_temporal,
                                         LivenessResult& liveness_result);

    /**
     * @brief Perform depth-based liveness analysis
     * @param depth_map Input depth map
     * @param face_box Face region
     * @param depth_score Output depth liveness score
     * @param depth_quality Output depth quality measure
     * @return True if depth liveness detected
     */
    bool analyzeDepthLiveness(const cv::Mat& depth_map,
                             const FaceBoundingBox& face_box,
                             float& depth_score,
                             float& depth_quality);

    /**
     * @brief Perform motion-based liveness analysis
     * @param current_frame Current RGB frame
     * @param face_box Current face box
     * @param motion_score Output motion liveness score
     * @param motion_pattern Output motion pattern analysis
     * @return True if motion liveness detected
     */
    bool analyzeMotionLiveness(const cv::Mat& current_frame,
                              const FaceBoundingBox& face_box,
                              float& motion_score,
                              std::string& motion_pattern);

    /**
     * @brief Perform texture-based liveness analysis
     * @param rgb_image Input RGB image
     * @param face_box Face region
     * @param texture_score Output texture liveness score
     * @param attack_type Output detected attack type
     * @return True if texture liveness detected
     */
    bool analyzeTextureLiveness(const cv::Mat& rgb_image,
                               const FaceBoundingBox& face_box,
                               float& texture_score,
                               std::string& attack_type);

    /**
     * @brief Detect eye blink patterns
     * @param rgb_frames Sequence of RGB frames
     * @param landmarks_sequence Sequence of landmarks
     * @param blink_frequency Output blink frequency (Hz)
     * @param blink_quality Output blink quality measure
     * @return True if natural blink patterns detected
     */
    bool detectBlinkPatterns(const std::vector<cv::Mat>& rgb_frames,
                            const std::vector<FacialLandmarks>& landmarks_sequence,
                            float& blink_frequency,
                            float& blink_quality);

    /**
     * @brief Analyze pulse/blood flow from face regions
     * @param rgb_frames Sequence of RGB frames
     * @param face_boxes Sequence of face detections
     * @param pulse_rate Output estimated pulse rate (BPM)
     * @param pulse_confidence Output pulse confidence
     * @return True if pulse detected
     */
    bool analyzePulseSignal(const std::vector<cv::Mat>& rgb_frames,
                           const std::vector<FaceBoundingBox>& face_boxes,
                           float& pulse_rate,
                           float& pulse_confidence);

    /**
     * @brief Perform frequency domain analysis for print detection
     * @param face_image Face region image
     * @param frequency_score Output frequency analysis score
     * @return True if real face (not print) detected
     */
    bool analyzeFrequencyDomain(const cv::Mat& face_image,
                               float& frequency_score);

    /**
     * @brief Compute Local Binary Pattern features
     * @param face_image Face region image
     * @param lbp_features Output LBP feature vector
     * @param lbp_score Output LBP liveness score
     */
    void computeLBPFeatures(const cv::Mat& face_image,
                           std::vector<float>& lbp_features,
                           float& lbp_score);

    /**
     * @brief Fuse multiple liveness scores
     * @param depth_score Depth liveness score
     * @param motion_score Motion liveness score
     * @param texture_score Texture liveness score
     * @param temporal_score Temporal consistency score
     * @param weights Fusion weights
     * @return Fused liveness score
     */
    float fuseLivenessScores(float depth_score,
                            float motion_score,
                            float texture_score,
                            float temporal_score,
                            const std::vector<float>& weights);

    /**
     * @brief Validate face region for liveness analysis
     * @param rgb_image Input RGB image
     * @param face_box Face bounding box
     * @param quality_score Output face quality
     * @return True if face region is suitable for analysis
     */
    bool validateFaceRegion(const cv::Mat& rgb_image,
                           const FaceBoundingBox& face_box,
                           float& quality_score);

    /**
     * @brief Update frame buffers for temporal analysis
     * @param rgb_frame New RGB frame
     * @param depth_frame New depth frame
     * @param face_box New face detection
     */
    void updateFrameBuffers(const cv::Mat& rgb_frame,
                           const cv::Mat& depth_frame,
                           const FaceBoundingBox& face_box);

    /**
     * @brief Process challenge response
     * @param challenge_type Type of challenge
     * @param response_frames Frames captured during challenge
     * @param success_score Output success score
     * @return True if challenge was completed successfully
     */
    bool processChallengeResponse(const std::string& challenge_type,
                                 const std::vector<cv::Mat>& response_frames,
                                 float& success_score);

    /**
     * @brief Update performance statistics
     * @param processing_time Time taken for detection
     * @param is_live Whether result was live detection
     * @param confidence Detection confidence
     */
    void updateStatistics(double processing_time,
                         bool is_live,
                         float confidence);

    // Disable copy constructor and assignment
    LivenessDetector(const LivenessDetector&) = delete;
    LivenessDetector& operator=(const LivenessDetector&) = delete;

    // Enable move semantics
    LivenessDetector(LivenessDetector&&) noexcept;
    LivenessDetector& operator=(LivenessDetector&&) noexcept;
};

/**
 * @brief Liveness detection utilities and algorithms
 */
class LivenessUtils {
public:
    /**
     * @brief Compute image quality metrics for liveness
     * @param image Input image
     * @param sharpness_score Output sharpness score
     * @param contrast_score Output contrast score
     * @param brightness_score Output brightness score
     * @return Overall image quality score (0-1)
     */
    static float computeImageQuality(const cv::Mat& image,
                                    float& sharpness_score,
                                    float& contrast_score,
                                    float& brightness_score);

    /**
     * @brief Detect screen reflections and moire patterns
     * @param image Input image
     * @param reflection_score Output reflection detection score
     * @param moire_score Output moire pattern score
     * @return True if screen artifacts detected
     */
    static bool detectScreenArtifacts(const cv::Mat& image,
                                     float& reflection_score,
                                     float& moire_score);

    /**
     * @brief Compute optical flow between consecutive frames
     * @param prev_frame Previous frame
     * @param curr_frame Current frame
     * @param flow_vectors Output optical flow vectors
     * @param flow_magnitude Output flow magnitude
     */
    static void computeOpticalFlow(const cv::Mat& prev_frame,
                                  const cv::Mat& curr_frame,
                                  cv::Mat& flow_vectors,
                                  cv::Mat& flow_magnitude);

    /**
     * @brief Extract heart rate from face region color changes
     * @param face_frames Sequence of face region images
     * @param sampling_rate Frame sampling rate (fps)
     * @param heart_rate Output estimated heart rate (BPM)
     * @param confidence Output estimation confidence
     * @return True if heart rate successfully estimated
     */
    static bool extractHeartRate(const std::vector<cv::Mat>& face_frames,
                                float sampling_rate,
                                float& heart_rate,
                                float& confidence);

    /**
     * @brief Analyze 3D face structure from depth map
     * @param depth_map Input depth map
     * @param face_region Face region of interest
     * @param structure_score Output 3D structure quality
     * @param curvature_score Output surface curvature score
     * @return True if valid 3D face structure detected
     */
    static bool analyze3DStructure(const cv::Mat& depth_map,
                                  const cv::Rect& face_region,
                                  float& structure_score,
                                  float& curvature_score);
};

} // namespace face
} // namespace unlook