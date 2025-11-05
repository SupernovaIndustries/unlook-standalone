#include "unlook/calibration/PatternDetector.hpp"
#include "unlook/core/Logger.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>

namespace unlook {
namespace calibration {

PatternDetector::PatternDetector(const PatternConfig& config)
    : config_(config)
    , confidenceScore_(0.0f) {

    // Initialize ChArUco components if needed
    if (config_.type == PatternType::CHARUCO) {
        arucoDict_ = cv::aruco::getPredefinedDictionary(config_.arucoDict);

        charucoBoard_ = cv::aruco::CharucoBoard::create(
            config_.cols, config_.rows,
            config_.squareSizeMM, config_.arucoMarkerSizeMM,
            arucoDict_
        );

        detectorParams_ = cv::aruco::DetectorParameters::create();
        // Optimize for calibration
        detectorParams_->adaptiveThreshWinSizeMin = 3;
        detectorParams_->adaptiveThreshWinSizeMax = 23;
        detectorParams_->adaptiveThreshWinSizeStep = 4;
        detectorParams_->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
        detectorParams_->cornerRefinementWinSize = 5;
    }
}

bool PatternDetector::detect(const cv::Mat& image,
                            std::vector<cv::Point2f>& corners,
                            cv::Mat& overlayImage) {
    // Clear previous results
    corners.clear();
    confidenceScore_ = 0.0f;

    // Clone image for overlay
    if (image.channels() == 1) {
        cv::cvtColor(image, overlayImage, cv::COLOR_GRAY2BGR);
    } else {
        overlayImage = image.clone();
    }

    bool detected = false;

    switch (config_.type) {
        case PatternType::CHECKERBOARD:
            detected = detectCheckerboard(image, corners, overlayImage);
            break;
        case PatternType::CHARUCO:
            detected = detectCharuco(image, corners, overlayImage);
            break;
        case PatternType::CIRCLE_GRID:
            detected = detectCircleGrid(image, corners, overlayImage);
            break;
    }

    return detected;
}

bool PatternDetector::detectCheckerboard(const cv::Mat& image,
                                        std::vector<cv::Point2f>& corners,
                                        cv::Mat& overlayImage) {
    cv::Mat gray;
    if (image.channels() == 3) {
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    } else {
        gray = image;
    }

    // Board size for checkerboard is inner corners
    cv::Size boardSize(config_.cols - 1, config_.rows - 1);

    // Find checkerboard corners
    bool found = cv::findChessboardCorners(
        gray, boardSize, corners,
        cv::CALIB_CB_ADAPTIVE_THRESH |
        cv::CALIB_CB_NORMALIZE_IMAGE |
        cv::CALIB_CB_FAST_CHECK
    );

    if (found) {
        // Refine corner positions
        cv::TermCriteria criteria(
            cv::TermCriteria::EPS + cv::TermCriteria::COUNT,
            30, 0.1
        );
        cv::cornerSubPix(gray, corners, cv::Size(11, 11),
                        cv::Size(-1, -1), criteria);

        // Calculate confidence based on corner detection quality
        int expectedCorners = (config_.rows - 1) * (config_.cols - 1);
        confidenceScore_ = static_cast<float>(corners.size()) / expectedCorners;

        // Draw corners on overlay
        cv::drawChessboardCorners(overlayImage, boardSize, corners, found);

        // Add text overlay
        cv::putText(overlayImage, "Checkerboard: " + std::to_string(corners.size()) + " corners",
                   cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7,
                   cv::Scalar(0, 255, 0), 2);

        core::Logger::getInstance().debug("Checkerboard detected: " +
                                        std::to_string(corners.size()) + " corners");
    } else {
        cv::putText(overlayImage, "Checkerboard: Not detected",
                   cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7,
                   cv::Scalar(0, 0, 255), 2);
    }

    return found;
}

bool PatternDetector::detectCharuco(const cv::Mat& image,
                                   std::vector<cv::Point2f>& corners,
                                   cv::Mat& overlayImage) {
    cv::Mat gray;
    if (image.channels() == 3) {
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    } else {
        gray = image;
    }

    // Detect ArUco markers
    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> markerCorners;

    cv::aruco::detectMarkers(gray, arucoDict_, markerCorners, markerIds,
                            detectorParams_, cv::noArray());

    if (markerIds.empty()) {
        cv::putText(overlayImage, "ChArUco: No markers detected",
                   cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7,
                   cv::Scalar(0, 0, 255), 2);
        return false;
    }

    // Draw detected markers
    cv::aruco::drawDetectedMarkers(overlayImage, markerCorners, markerIds);

    // Interpolate ChArUco corners
    std::vector<int> charucoIds;
    cv::aruco::interpolateCornersCharuco(
        markerCorners, markerIds, gray, charucoBoard_,
        corners, charucoIds
    );

    if (corners.size() >= 4) {  // Minimum corners for calibration
        // Refine corners using corner subpix
        cv::TermCriteria criteria(
            cv::TermCriteria::EPS + cv::TermCriteria::COUNT,
            30, 0.01
        );
        cv::cornerSubPix(gray, corners, cv::Size(5, 5),
                        cv::Size(-1, -1), criteria);

        // Calculate confidence
        int expectedCorners = (config_.rows - 1) * (config_.cols - 1);
        confidenceScore_ = static_cast<float>(corners.size()) / expectedCorners;

        // Draw ChArUco corners
        cv::aruco::drawDetectedCornersCharuco(overlayImage, corners, charucoIds);

        // Add text overlay
        cv::putText(overlayImage,
                   "ChArUco: " + std::to_string(corners.size()) + " corners, " +
                   std::to_string(markerIds.size()) + " markers",
                   cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7,
                   cv::Scalar(0, 255, 0), 2);

        core::Logger::getInstance().debug("ChArUco detected: " +
                                        std::to_string(corners.size()) + " corners, " +
                                        std::to_string(markerIds.size()) + " markers");

        return true;
    } else {
        cv::putText(overlayImage,
                   "ChArUco: Insufficient corners (" + std::to_string(corners.size()) + ")",
                   cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7,
                   cv::Scalar(0, 165, 255), 2);  // Orange for warning
        return false;
    }
}

bool PatternDetector::detectCircleGrid(const cv::Mat& image,
                                      std::vector<cv::Point2f>& corners,
                                      cv::Mat& overlayImage) {
    cv::Mat gray;
    if (image.channels() == 3) {
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    } else {
        gray = image;
    }

    cv::Size patternSize(config_.cols, config_.rows);

    // Try asymmetric circle grid first
    bool found = cv::findCirclesGrid(
        gray, patternSize, corners,
        cv::CALIB_CB_ASYMMETRIC_GRID | cv::CALIB_CB_CLUSTERING
    );

    if (!found) {
        // Try symmetric circle grid
        found = cv::findCirclesGrid(
            gray, patternSize, corners,
            cv::CALIB_CB_SYMMETRIC_GRID | cv::CALIB_CB_CLUSTERING
        );
    }

    if (found) {
        // Calculate confidence
        int expectedCorners = config_.rows * config_.cols;
        confidenceScore_ = static_cast<float>(corners.size()) / expectedCorners;

        // Draw corners
        cv::drawChessboardCorners(overlayImage, patternSize, corners, found);

        cv::putText(overlayImage,
                   "Circle Grid: " + std::to_string(corners.size()) + " circles",
                   cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7,
                   cv::Scalar(0, 255, 0), 2);

        core::Logger::getInstance().debug("Circle grid detected: " +
                                        std::to_string(corners.size()) + " circles");
    } else {
        cv::putText(overlayImage, "Circle Grid: Not detected",
                   cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7,
                   cv::Scalar(0, 0, 255), 2);
    }

    return found;
}

int PatternDetector::getExpectedCorners() const {
    switch (config_.type) {
        case PatternType::CHECKERBOARD:
            return (config_.rows - 1) * (config_.cols - 1);
        case PatternType::CHARUCO:
            return (config_.rows - 1) * (config_.cols - 1);
        case PatternType::CIRCLE_GRID:
            return config_.rows * config_.cols;
        default:
            return 0;
    }
}

} // namespace calibration
} // namespace unlook