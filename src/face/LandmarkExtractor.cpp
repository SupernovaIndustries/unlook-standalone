#include "unlook/face/LandmarkExtractor.hpp"
#include "unlook/face/FaceException.hpp"
#include "unlook/core/logging.hpp"
#include <opencv2/objdetect.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <algorithm>
#include <numeric>
#include <chrono>
#include <cmath>

#ifdef __ARM_NEON
#include <arm_neon.h>
#endif

namespace unlook {
namespace face {

// Standard 68-point facial landmark indices for semantic groups
static const std::map<std::string, std::vector<int>> LANDMARK_68_GROUPS = {
    {"jaw",        {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16}},
    {"right_eyebrow", {17, 18, 19, 20, 21}},
    {"left_eyebrow",  {22, 23, 24, 25, 26}},
    {"nose_bridge",   {27, 28, 29, 30}},
    {"nose_tip",      {31, 32, 33, 34, 35}},
    {"right_eye",     {36, 37, 38, 39, 40, 41}},
    {"left_eye",      {42, 43, 44, 45, 46, 47}},
    {"outer_lip",     {48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59}},
    {"inner_lip",     {60, 61, 62, 63, 64, 65, 66, 67}}
};

static const std::vector<std::string> LANDMARK_68_NAMES = {
    "jaw_0", "jaw_1", "jaw_2", "jaw_3", "jaw_4", "jaw_5", "jaw_6", "jaw_7", "jaw_8",
    "jaw_9", "jaw_10", "jaw_11", "jaw_12", "jaw_13", "jaw_14", "jaw_15", "jaw_16",
    "right_eyebrow_0", "right_eyebrow_1", "right_eyebrow_2", "right_eyebrow_3", "right_eyebrow_4",
    "left_eyebrow_0", "left_eyebrow_1", "left_eyebrow_2", "left_eyebrow_3", "left_eyebrow_4",
    "nose_bridge_0", "nose_bridge_1", "nose_bridge_2", "nose_bridge_3",
    "nose_tip_0", "nose_tip_1", "nose_tip_2", "nose_tip_3", "nose_tip_4",
    "right_eye_0", "right_eye_1", "right_eye_2", "right_eye_3", "right_eye_4", "right_eye_5",
    "left_eye_0", "left_eye_1", "left_eye_2", "left_eye_3", "left_eye_4", "left_eye_5",
    "outer_lip_0", "outer_lip_1", "outer_lip_2", "outer_lip_3", "outer_lip_4", "outer_lip_5",
    "outer_lip_6", "outer_lip_7", "outer_lip_8", "outer_lip_9", "outer_lip_10", "outer_lip_11",
    "inner_lip_0", "inner_lip_1", "inner_lip_2", "inner_lip_3", "inner_lip_4", "inner_lip_5",
    "inner_lip_6", "inner_lip_7"
};

// Standard 3D model points for pose estimation (approximate positions in face coordinate system)
static const std::vector<cv::Point3f> MODEL_POINTS_68 = {
    {0.0f, 0.0f, 0.0f},        // Nose tip (reference point)
    {0.0f, -330.0f, -65.0f},   // Chin
    {-225.0f, 170.0f, -135.0f}, // Left eye left corner
    {225.0f, 170.0f, -135.0f},  // Right eye right corner
    {-150.0f, -150.0f, -125.0f}, // Left mouth corner
    {150.0f, -150.0f, -125.0f},  // Right mouth corner
};

/**
 * @brief Private implementation class for LandmarkExtractor
 */
class LandmarkExtractor::Impl {
public:
    // OpenCV face detector and landmark predictor
    cv::CascadeClassifier face_cascade_;
    cv::face::Facemark face_mark_;

    // Configuration
    LandmarkExtractionConfig config_;

    // Camera calibration data
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;
    bool has_camera_calibration_ = false;

    // Error handling
    std::string last_error_;

    // Banking mode state
    bool banking_mode_ = false;

    Impl() = default;

    /**
     * @brief Initialize landmark detection models
     */
    FaceResultCode initializeModels(const std::string& model_path, LandmarkModel model_type) {
        try {
            // Load OpenCV face detection cascade
            std::string cascade_path = cv::samples::findFile("haarcascade_frontalface_alt.xml");
            if (!face_cascade_.load(cascade_path)) {
                // Try alternative paths
                cascade_path = "/usr/share/opencv4/haarcascades/haarcascade_frontalface_alt.xml";
                if (!face_cascade_.load(cascade_path)) {
                    last_error_ = "Failed to load face detection cascade";
                    return FaceResultCode::ERROR_LANDMARK_DETECTION_FAILED;
                }
            }

            // Initialize landmark detector based on model type
            if (model_type == LandmarkModel::LANDMARKS_68) {
                face_mark_ = cv::face::FacemarkLBF::create();

                // Try to load model file
                if (!model_path.empty() && !face_mark_->loadModel(model_path)) {
                    // Try default model path
                    std::string default_path = cv::samples::findFile("lbfmodel.yaml");
                    if (!face_mark_->loadModel(default_path)) {
                        last_error_ = "Failed to load 68-point landmark model: " + model_path;
                        return FaceResultCode::ERROR_LANDMARK_DETECTION_FAILED;
                    }
                }
            } else {
                last_error_ = "Unsupported landmark model type";
                return FaceResultCode::ERROR_LANDMARK_DETECTION_FAILED;
            }

            return FaceResultCode::SUCCESS;

        } catch (const std::exception& e) {
            last_error_ = "Exception during model initialization: " + std::string(e.what());
            return FaceResultCode::ERROR_LANDMARK_DETECTION_FAILED;
        }
    }

    /**
     * @brief Detect facial landmarks in 2D image
     */
    FaceResultCode detectLandmarks2D(const cv::Mat& image, const FaceBoundingBox& face_box,
                                    std::vector<LandmarkPoint2D>& landmarks_2d) {
        auto start_time = std::chrono::high_resolution_clock::now();

        try {
            landmarks_2d.clear();

            // Convert face box to OpenCV rectangle
            cv::Rect face_rect(static_cast<int>(face_box.bbox.x),
                              static_cast<int>(face_box.bbox.y),
                              static_cast<int>(face_box.bbox.width),
                              static_cast<int>(face_box.bbox.height));

            // Ensure face rectangle is within image bounds
            face_rect &= cv::Rect(0, 0, image.cols, image.rows);
            if (face_rect.width < 50 || face_rect.height < 50) {
                last_error_ = "Face region too small for landmark detection";
                return FaceResultCode::ERROR_FACE_TOO_SMALL;
            }

            // Convert to grayscale if needed
            cv::Mat gray_image;
            if (image.channels() == 3) {
                cv::cvtColor(image, gray_image, cv::COLOR_BGR2GRAY);
            } else {
                gray_image = image.clone();
            }

            // Extract face region for better accuracy
            cv::Mat face_roi = gray_image(face_rect);

            // Enhance contrast for better landmark detection
            cv::Mat enhanced_face;
            cv::equalizeHist(face_roi, enhanced_face);

            // Detect landmarks using OpenCV facemark
            std::vector<cv::Rect> faces = {face_rect};
            std::vector<std::vector<cv::Point2f>> landmarks;

            bool success = face_mark_->fit(gray_image, faces, landmarks);
            if (!success || landmarks.empty() || landmarks[0].empty()) {
                last_error_ = "Failed to detect facial landmarks";
                return FaceResultCode::ERROR_LANDMARK_DETECTION_FAILED;
            }

            // Convert to our landmark format with confidence estimation
            const auto& detected_points = landmarks[0];
            landmarks_2d.reserve(detected_points.size());

            for (size_t i = 0; i < detected_points.size(); ++i) {
                float confidence = estimateLandmarkConfidence(detected_points[i], face_roi, face_rect);
                landmarks_2d.emplace_back(detected_points[i].x, detected_points[i].y, confidence);
            }

            // Validate landmark count for banking mode
            if (banking_mode_ && landmarks_2d.size() != 68) {
                last_error_ = "Banking mode requires exactly 68 landmarks";
                return FaceResultCode::ERROR_INSUFFICIENT_LANDMARKS;
            }

            return FaceResultCode::SUCCESS;

        } catch (const std::exception& e) {
            last_error_ = "Exception during 2D landmark detection: " + std::string(e.what());
            return FaceResultCode::ERROR_LANDMARK_DETECTION_FAILED;
        }
    }

    /**
     * @brief Convert 2D landmarks to 3D using depth information
     */
    size_t convert2DTo3D(const std::vector<LandmarkPoint2D>& landmarks_2d,
                        const cv::Mat& depth_map,
                        const cv::Mat& camera_matrix,
                        std::vector<LandmarkPoint3D>& landmarks_3d) {
        landmarks_3d.clear();
        landmarks_3d.reserve(landmarks_2d.size());

        size_t successful_conversions = 0;

        // Camera intrinsic parameters
        float fx = camera_matrix.at<double>(0, 0);
        float fy = camera_matrix.at<double>(1, 1);
        float cx = camera_matrix.at<double>(0, 2);
        float cy = camera_matrix.at<double>(1, 2);

        for (const auto& point_2d : landmarks_2d) {
            int x = static_cast<int>(std::round(point_2d.x));
            int y = static_cast<int>(std::round(point_2d.y));

            // Check if point is within depth map bounds
            if (x >= 0 && x < depth_map.cols && y >= 0 && y < depth_map.rows) {
                float depth_mm = depth_map.at<float>(y, x);

                // Validate depth value
                if (depth_mm > config_.stereo_confidence_threshold &&
                    depth_mm >= 100.0f && depth_mm <= 1000.0f) {

                    // Convert pixel coordinates to 3D using pinhole camera model
                    float x_3d = (x - cx) * depth_mm / fx;
                    float y_3d = (y - cy) * depth_mm / fy;
                    float z_3d = depth_mm;

                    // Estimate depth quality from surrounding pixels
                    float depth_quality = estimateDepthQuality(depth_map, x, y);

                    landmarks_3d.emplace_back(x_3d, y_3d, z_3d, point_2d.confidence, depth_quality);
                    successful_conversions++;
                } else {
                    // Add invalid 3D point to maintain correspondence
                    landmarks_3d.emplace_back(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
                }
            } else {
                // Point outside depth map bounds
                landmarks_3d.emplace_back(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
            }
        }

        return successful_conversions;
    }

    /**
     * @brief Estimate landmark confidence based on local image features
     */
    float estimateLandmarkConfidence(const cv::Point2f& point, const cv::Mat& face_roi,
                                   const cv::Rect& face_rect) {
        // Adjust point coordinates to face ROI
        cv::Point2f roi_point(point.x - face_rect.x, point.y - face_rect.y);

        // Check if point is within ROI bounds
        if (roi_point.x < 5 || roi_point.x >= face_roi.cols - 5 ||
            roi_point.y < 5 || roi_point.y >= face_roi.rows - 5) {
            return 0.3f; // Low confidence for boundary points
        }

        // Compute local gradient magnitude as confidence measure
        cv::Mat grad_x, grad_y;
        cv::Sobel(face_roi, grad_x, CV_32F, 1, 0, 3);
        cv::Sobel(face_roi, grad_y, CV_32F, 0, 1, 3);

        // Sample gradient around landmark point
        int x = static_cast<int>(roi_point.x);
        int y = static_cast<int>(roi_point.y);

        float grad_mag = 0.0f;
        int sample_count = 0;

        for (int dy = -2; dy <= 2; dy++) {
            for (int dx = -2; dx <= 2; dx++) {
                int px = x + dx, py = y + dy;
                if (px >= 0 && px < face_roi.cols && py >= 0 && py < face_roi.rows) {
                    float gx = grad_x.at<float>(py, px);
                    float gy = grad_y.at<float>(py, px);
                    grad_mag += std::sqrt(gx * gx + gy * gy);
                    sample_count++;
                }
            }
        }

        if (sample_count > 0) {
            grad_mag /= sample_count;
            // Normalize to 0-1 range (typical gradient magnitude range 0-100)
            return std::min(1.0f, grad_mag / 50.0f);
        }

        return 0.5f; // Default medium confidence
    }

    /**
     * @brief Estimate depth quality from surrounding pixels
     */
    float estimateDepthQuality(const cv::Mat& depth_map, int x, int y) {
        float center_depth = depth_map.at<float>(y, x);
        if (center_depth <= 0) return 0.0f;

        float depth_variance = 0.0f;
        float valid_count = 0.0f;
        const int radius = 3;

        for (int dy = -radius; dy <= radius; dy++) {
            for (int dx = -radius; dx <= radius; dx++) {
                int px = x + dx, py = y + dy;
                if (px >= 0 && px < depth_map.cols && py >= 0 && py < depth_map.rows) {
                    float neighbor_depth = depth_map.at<float>(py, px);
                    if (neighbor_depth > 0) {
                        float diff = neighbor_depth - center_depth;
                        depth_variance += diff * diff;
                        valid_count += 1.0f;
                    }
                }
            }
        }

        if (valid_count > 0) {
            depth_variance /= valid_count;
            float std_dev = std::sqrt(depth_variance);

            // Quality decreases with higher variance
            // Good quality: std_dev < 2mm, Poor quality: std_dev > 10mm
            float quality = std::max(0.0f, std::min(1.0f, (10.0f - std_dev) / 8.0f));
            return quality;
        }

        return 0.5f; // Default medium quality
    }

    /**
     * @brief Validate landmark geometry using statistical analysis
     */
    float validateGeometry(const FacialLandmarks& landmarks, const cv::Mat& image) {
        if (landmarks.points2D.size() != 68) {
            return 0.0f; // Invalid landmark count
        }

        float geometry_score = 1.0f;

        // Check inter-ocular distance (should be reasonable)
        auto left_eye_center = computeGroupCenter(landmarks.points2D, LANDMARK_68_GROUPS.at("left_eye"));
        auto right_eye_center = computeGroupCenter(landmarks.points2D, LANDMARK_68_GROUPS.at("right_eye"));

        float iod = cv::norm(left_eye_center - right_eye_center);
        if (iod < 30 || iod > 200) { // Typical range 50-150 pixels
            geometry_score *= 0.7f;
        }

        // Check nose-to-mouth distance
        auto nose_tip = landmarks.points2D[33]; // Nose tip
        auto mouth_center = computeGroupCenter(landmarks.points2D, LANDMARK_68_GROUPS.at("outer_lip"));

        float nose_mouth_dist = cv::norm(cv::Point2f(nose_tip.x, nose_tip.y) - mouth_center);
        if (nose_mouth_dist < 20 || nose_mouth_dist > 100) {
            geometry_score *= 0.8f;
        }

        // Check facial symmetry
        float symmetry_score = computeFacialSymmetry(landmarks.points2D);
        geometry_score *= (0.5f + 0.5f * symmetry_score);

        // Check landmark spread (should cover reasonable area)
        cv::Rect bounding_rect = computeLandmarkBoundingRect(landmarks.points2D);
        float area_ratio = static_cast<float>(bounding_rect.area()) / (image.cols * image.rows);
        if (area_ratio < 0.01f || area_ratio > 0.5f) {
            geometry_score *= 0.6f;
        }

        return std::max(0.0f, std::min(1.0f, geometry_score));
    }

    /**
     * @brief Compute center point of landmark group
     */
    cv::Point2f computeGroupCenter(const std::vector<LandmarkPoint2D>& landmarks,
                                  const std::vector<int>& indices) {
        cv::Point2f center(0, 0);
        for (int idx : indices) {
            if (idx < static_cast<int>(landmarks.size())) {
                center.x += landmarks[idx].x;
                center.y += landmarks[idx].y;
            }
        }
        center.x /= indices.size();
        center.y /= indices.size();
        return center;
    }

    /**
     * @brief Compute facial symmetry score
     */
    float computeFacialSymmetry(const std::vector<LandmarkPoint2D>& landmarks) {
        // Use symmetric landmark pairs to compute symmetry
        std::vector<std::pair<int, int>> symmetric_pairs = {
            {17, 26}, {18, 25}, {19, 24}, {20, 23}, {21, 22}, // Eyebrows
            {36, 45}, {37, 44}, {38, 43}, {39, 42}, {40, 47}, {41, 46}, // Eyes
            {31, 35}, {32, 34}, // Nose
            {48, 54}, {49, 53}, {50, 52}, {51, 51}, // Outer lip
            {60, 64}, {61, 63}, {62, 62} // Inner lip
        };

        // Compute face center line
        float face_center_x = (landmarks[27].x + landmarks[30].x + landmarks[33].x) / 3.0f; // Nose bridge average

        float symmetry_error = 0.0f;
        int pair_count = 0;

        for (const auto& pair : symmetric_pairs) {
            int left_idx = pair.first;
            int right_idx = pair.second;

            if (left_idx < static_cast<int>(landmarks.size()) &&
                right_idx < static_cast<int>(landmarks.size())) {

                float left_dist = std::abs(landmarks[left_idx].x - face_center_x);
                float right_dist = std::abs(landmarks[right_idx].x - face_center_x);

                float y_diff = std::abs(landmarks[left_idx].y - landmarks[right_idx].y);

                // Both distance asymmetry and height difference contribute to error
                symmetry_error += std::abs(left_dist - right_dist) + y_diff * 0.5f;
                pair_count++;
            }
        }

        if (pair_count > 0) {
            symmetry_error /= pair_count;
            // Convert error to score (lower error = higher score)
            return std::max(0.0f, std::min(1.0f, (20.0f - symmetry_error) / 20.0f));
        }

        return 0.5f; // Default medium symmetry
    }

    /**
     * @brief Compute bounding rectangle of landmarks
     */
    cv::Rect computeLandmarkBoundingRect(const std::vector<LandmarkPoint2D>& landmarks) {
        if (landmarks.empty()) return cv::Rect();

        float min_x = landmarks[0].x, max_x = landmarks[0].x;
        float min_y = landmarks[0].y, max_y = landmarks[0].y;

        for (const auto& point : landmarks) {
            min_x = std::min(min_x, point.x);
            max_x = std::max(max_x, point.x);
            min_y = std::min(min_y, point.y);
            max_y = std::max(max_y, point.y);
        }

        return cv::Rect(static_cast<int>(min_x), static_cast<int>(min_y),
                       static_cast<int>(max_x - min_x), static_cast<int>(max_y - min_y));
    }
};

// LandmarkExtractionConfig validation
bool LandmarkExtractionConfig::validate() const {
    if (confidence_threshold < 0.0f || confidence_threshold > 1.0f) return false;
    if (min_landmark_visibility < 0.0f || min_landmark_visibility > 1.0f) return false;
    if (max_landmark_deviation < 0.5f || max_landmark_deviation > 5.0f) return false;
    if (depth_weight < 0.0f || depth_weight > 1.0f) return false;
    if (stereo_confidence_threshold < 0.0f || stereo_confidence_threshold > 1.0f) return false;
    if (banking_confidence_threshold < 0.8f || banking_confidence_threshold > 1.0f) return false;
    if (max_threads < 1 || max_threads > 16) return false;
    return true;
}

std::string LandmarkExtractionConfig::toString() const {
    return "LandmarkExtractionConfig{" +
           std::string("model=") + (model_type == LandmarkModel::LANDMARKS_68 ? "68pt" : "unknown") +
           ", confidence=" + std::to_string(confidence_threshold) +
           ", 3d_enabled=" + (enable_3d_estimation ? "true" : "false") +
           ", banking_mode=" + (banking_mode ? "true" : "false") + "}";
}

// LandmarkExtractor implementation
LandmarkExtractor::LandmarkExtractor() : pImpl(std::make_unique<Impl>()) {
    // Default configuration
    LandmarkExtractionConfig default_config;
    pImpl->config_ = default_config;
}

LandmarkExtractor::LandmarkExtractor(const LandmarkExtractionConfig& config)
    : pImpl(std::make_unique<Impl>()) {
    pImpl->config_ = config;
}

LandmarkExtractor::~LandmarkExtractor() = default;

LandmarkExtractor::LandmarkExtractor(LandmarkExtractor&&) noexcept = default;
LandmarkExtractor& LandmarkExtractor::operator=(LandmarkExtractor&&) noexcept = default;

FaceResultCode LandmarkExtractor::initialize(const std::string& model_path, LandmarkModel model_type) {
    return pImpl->initializeModels(model_path, model_type);
}

bool LandmarkExtractor::isInitialized() const {
    return !pImpl->face_cascade_.empty() && !pImpl->face_mark_.empty();
}

FaceResultCode LandmarkExtractor::extractLandmarks2D(const cv::Mat& image,
                                                    const FaceBoundingBox& face_box,
                                                    FacialLandmarks& landmarks) {
    auto start_time = std::chrono::high_resolution_clock::now();

    landmarks = FacialLandmarks();
    landmarks.model = pImpl->config_.model_type;

    FaceResultCode result = pImpl->detectLandmarks2D(image, face_box, landmarks.points2D);

    if (result == FaceResultCode::SUCCESS) {
        // Compute overall confidence
        float total_confidence = 0.0f;
        for (const auto& point : landmarks.points2D) {
            total_confidence += point.confidence;
        }
        landmarks.overall_confidence = landmarks.points2D.empty() ? 0.0f :
                                     total_confidence / landmarks.points2D.size();

        // Validate geometry if enabled
        if (pImpl->config_.enable_geometric_validation) {
            landmarks.geometric_consistency = pImpl->validateGeometry(landmarks, image);
        } else {
            landmarks.geometric_consistency = 1.0f;
        }

        // Banking mode validation
        if (pImpl->banking_mode_) {
            if (landmarks.overall_confidence < pImpl->config_.banking_confidence_threshold) {
                return FaceResultCode::ERROR_LANDMARK_QUALITY_LOW;
            }
            if (pImpl->config_.require_all_landmarks && landmarks.points2D.size() != 68) {
                return FaceResultCode::ERROR_INSUFFICIENT_LANDMARKS;
            }
        }
    }

    auto end_time = std::chrono::high_resolution_clock::now();
    landmarks.detection_time_ms = std::chrono::duration<double, std::milli>(end_time - start_time).count();

    // Update statistics
    updateStatistics(landmarks.detection_time_ms, result == FaceResultCode::SUCCESS, landmarks.overall_confidence);

    return result;
}

FaceResultCode LandmarkExtractor::extractLandmarks3D(const cv::Mat& rgb_image,
                                                    const cv::Mat& depth_map,
                                                    const FaceBoundingBox& face_box,
                                                    FacialLandmarks& landmarks) {
    auto start_time = std::chrono::high_resolution_clock::now();

    // First extract 2D landmarks
    FaceResultCode result = extractLandmarks2D(rgb_image, face_box, landmarks);
    if (result != FaceResultCode::SUCCESS) {
        return result;
    }

    // Convert to 3D if camera calibration is available
    if (pImpl->has_camera_calibration_ && !depth_map.empty()) {
        size_t successful_3d = pImpl->convert2DTo3D(landmarks.points2D, depth_map,
                                                   pImpl->camera_matrix_, landmarks.points3D);

        landmarks.depth_coverage = landmarks.points2D.empty() ? 0.0f :
                                 static_cast<float>(successful_3d) / landmarks.points2D.size();

        // Banking mode 3D validation
        if (pImpl->banking_mode_ && landmarks.depth_coverage < 0.8f) {
            return FaceResultCode::ERROR_DEPTH_DATA_INVALID;
        }
    } else {
        landmarks.depth_coverage = 0.0f;
        if (pImpl->config_.enable_3d_estimation) {
            pImpl->last_error_ = "3D estimation requires camera calibration and depth map";
            return FaceResultCode::ERROR_DEPTH_DATA_INVALID;
        }
    }

    auto end_time = std::chrono::high_resolution_clock::now();
    landmarks.detection_time_ms = std::chrono::duration<double, std::milli>(end_time - start_time).count();

    return FaceResultCode::SUCCESS;
}

FaceResultCode LandmarkExtractor::refineLandmarksWithDepth(const FacialLandmarks& landmarks,
                                                          const cv::Mat& depth_map,
                                                          const cv::Mat& confidence_map,
                                                          FacialLandmarks& refined_landmarks) {
    if (!pImpl->has_camera_calibration_) {
        pImpl->last_error_ = "Camera calibration required for depth refinement";
        return FaceResultCode::ERROR_DEPTH_DATA_INVALID;
    }

    refined_landmarks = landmarks;

    // Refine 3D positions using depth confidence
    if (landmarks.has3D() && !confidence_map.empty()) {
        for (size_t i = 0; i < refined_landmarks.points3D.size(); ++i) {
            auto& point_3d = refined_landmarks.points3D[i];
            const auto& point_2d = landmarks.points2D[i];

            int x = static_cast<int>(std::round(point_2d.x));
            int y = static_cast<int>(std::round(point_2d.y));

            if (x >= 0 && x < confidence_map.cols && y >= 0 && y < confidence_map.rows) {
                float depth_confidence = confidence_map.at<float>(y, x);

                // Weight the existing confidence with depth confidence
                point_3d.confidence = (point_3d.confidence + depth_confidence) * 0.5f;
                point_3d.depth_quality = depth_confidence;
            }
        }

        // Recompute overall metrics
        float total_confidence = 0.0f;
        float valid_3d_count = 0.0f;

        for (const auto& point : refined_landmarks.points3D) {
            total_confidence += point.confidence;
            if (point.depth_quality > pImpl->config_.stereo_confidence_threshold) {
                valid_3d_count += 1.0f;
            }
        }

        refined_landmarks.overall_confidence = refined_landmarks.points3D.empty() ? 0.0f :
                                             total_confidence / refined_landmarks.points3D.size();
        refined_landmarks.depth_coverage = refined_landmarks.points3D.empty() ? 0.0f :
                                          valid_3d_count / refined_landmarks.points3D.size();
    }

    return FaceResultCode::SUCCESS;
}

void LandmarkExtractor::setCameraCalibration(const cv::Mat& camera_matrix, const cv::Mat& dist_coeffs) {
    pImpl->camera_matrix_ = camera_matrix.clone();
    pImpl->dist_coeffs_ = dist_coeffs.clone();
    pImpl->has_camera_calibration_ = true;
}

void LandmarkExtractor::setBankingMode(bool enable, bool require_all_landmarks, float min_confidence) {
    pImpl->banking_mode_ = enable;
    pImpl->config_.banking_mode = enable;
    pImpl->config_.require_all_landmarks = require_all_landmarks;
    pImpl->config_.banking_confidence_threshold = min_confidence;
}

FaceResultCode LandmarkExtractor::setConfiguration(const LandmarkExtractionConfig& config) {
    if (!config.validate()) {
        pImpl->last_error_ = "Invalid landmark extraction configuration";
        return FaceResultCode::ERROR_TEMPLATE_INVALID;
    }
    pImpl->config_ = config;
    return FaceResultCode::SUCCESS;
}

LandmarkExtractionConfig LandmarkExtractor::getConfiguration() const {
    return pImpl->config_;
}

std::string LandmarkExtractor::getLastError() const {
    return pImpl->last_error_;
}

// Static utility methods
std::vector<LandmarkModel> LandmarkExtractor::getSupportedModels() {
    return {LandmarkModel::LANDMARKS_68};
}

std::vector<std::string> LandmarkExtractor::getLandmarkNames(LandmarkModel model) {
    if (model == LandmarkModel::LANDMARKS_68) {
        return LANDMARK_68_NAMES;
    }
    return {};
}

std::map<std::string, std::vector<int>> LandmarkExtractor::getLandmarkGroups(LandmarkModel model) {
    if (model == LandmarkModel::LANDMARKS_68) {
        return LANDMARK_68_GROUPS;
    }
    return {};
}

bool LandmarkExtractor::validateModel(const std::string& model_path, LandmarkModel model_type) {
    if (model_path.empty()) return false;

    // Check if file exists and is readable
    std::ifstream file(model_path);
    if (!file.good()) return false;

    // Additional validation could be added here for specific model formats
    return true;
}

void LandmarkExtractor::updateStatistics(double extraction_time, bool success, float confidence) {
    total_extractions_.fetch_add(1);
    if (success) {
        successful_extractions_.fetch_add(1);
    }

    // Update running averages (simplified approach)
    total_extraction_time_.store(total_extraction_time_.load() + extraction_time);
    total_confidence_.store(total_confidence_.load() + confidence);
}

void LandmarkExtractor::getStatistics(double& avg_extraction_time, size_t& total_extractions,
                                     float& success_rate, float& avg_landmark_confidence) const {
    std::lock_guard<std::mutex> lock(stats_mutex_);

    total_extractions = total_extractions_.load();
    size_t successful = successful_extractions_.load();

    success_rate = (total_extractions > 0) ?
                  static_cast<float>(successful) / total_extractions : 0.0f;

    avg_extraction_time = (total_extractions > 0) ?
                         total_extraction_time_.load() / total_extractions : 0.0;

    avg_landmark_confidence = (total_extractions > 0) ?
                             total_confidence_.load() / total_extractions : 0.0f;
}

void LandmarkExtractor::resetStatistics() {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    total_extractions_.store(0);
    successful_extractions_.store(0);
    total_extraction_time_.store(0.0);
    total_confidence_.store(0.0);
}

// LandmarkUtils implementation
void LandmarkUtils::drawLandmarks(cv::Mat& image, const FacialLandmarks& landmarks,
                                 const cv::Scalar& color, int radius, bool draw_connections) {
    // Draw landmark points
    for (const auto& point : landmarks.points2D) {
        cv::Point2i pixel_point(static_cast<int>(point.x), static_cast<int>(point.y));

        // Color intensity based on confidence
        cv::Scalar point_color = color * point.confidence;
        cv::circle(image, pixel_point, radius, point_color, -1);
    }

    // Draw connections if requested
    if (draw_connections && landmarks.points2D.size() == 68) {
        auto groups = LandmarkExtractor::getLandmarkGroups(LandmarkModel::LANDMARKS_68);

        // Draw group connections
        for (const auto& group : groups) {
            const auto& indices = group.second;
            for (size_t i = 1; i < indices.size(); ++i) {
                cv::Point2i p1(static_cast<int>(landmarks.points2D[indices[i-1]].x),
                              static_cast<int>(landmarks.points2D[indices[i-1]].y));
                cv::Point2i p2(static_cast<int>(landmarks.points2D[indices[i]].x),
                              static_cast<int>(landmarks.points2D[indices[i]].y));
                cv::line(image, p1, p2, color * 0.7, 1);
            }
        }
    }
}

void LandmarkUtils::computeFacialMeasurements(const FacialLandmarks& landmarks,
                                            float& interpupillary_distance,
                                            float& face_width, float& face_height,
                                            bool use_3d) {
    if (landmarks.points2D.size() != 68) {
        interpupillary_distance = face_width = face_height = 0.0f;
        return;
    }

    if (use_3d && landmarks.has3D()) {
        // 3D measurements
        const auto& left_eye = landmarks.points3D[36];   // Left eye outer corner
        const auto& right_eye = landmarks.points3D[45];  // Right eye outer corner

        interpupillary_distance = std::sqrt(
            (left_eye.x - right_eye.x) * (left_eye.x - right_eye.x) +
            (left_eye.y - right_eye.y) * (left_eye.y - right_eye.y) +
            (left_eye.z - right_eye.z) * (left_eye.z - right_eye.z)
        );

        // Face width (jaw corners)
        const auto& jaw_left = landmarks.points3D[0];
        const auto& jaw_right = landmarks.points3D[16];
        face_width = std::sqrt(
            (jaw_left.x - jaw_right.x) * (jaw_left.x - jaw_right.x) +
            (jaw_left.y - jaw_right.y) * (jaw_left.y - jaw_right.y) +
            (jaw_left.z - jaw_right.z) * (jaw_left.z - jaw_right.z)
        );

        // Face height (forehead to chin approximation)
        const auto& forehead = landmarks.points3D[27];  // Nose bridge (approximate forehead)
        const auto& chin = landmarks.points3D[8];       // Chin center
        face_height = std::sqrt(
            (forehead.x - chin.x) * (forehead.x - chin.x) +
            (forehead.y - chin.y) * (forehead.y - chin.y) +
            (forehead.z - chin.z) * (forehead.z - chin.z)
        );
    } else {
        // 2D measurements (in pixels)
        const auto& left_eye = landmarks.points2D[36];
        const auto& right_eye = landmarks.points2D[45];

        interpupillary_distance = std::sqrt(
            (left_eye.x - right_eye.x) * (left_eye.x - right_eye.x) +
            (left_eye.y - right_eye.y) * (left_eye.y - right_eye.y)
        );

        const auto& jaw_left = landmarks.points2D[0];
        const auto& jaw_right = landmarks.points2D[16];
        face_width = std::sqrt(
            (jaw_left.x - jaw_right.x) * (jaw_left.x - jaw_right.x) +
            (jaw_left.y - jaw_right.y) * (jaw_left.y - jaw_right.y)
        );

        const auto& forehead = landmarks.points2D[27];
        const auto& chin = landmarks.points2D[8];
        face_height = std::sqrt(
            (forehead.x - chin.x) * (forehead.x - chin.x) +
            (forehead.y - chin.y) * (forehead.y - chin.y)
        );
    }
}

} // namespace face
} // namespace unlook