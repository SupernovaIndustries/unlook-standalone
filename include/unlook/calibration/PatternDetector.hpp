#pragma once

#include "StereoCalibrationProcessor.hpp"  // For PatternType and PatternConfig
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <memory>
#include <vector>

namespace unlook {
namespace calibration {

class PatternDetector {
public:
    explicit PatternDetector(const PatternConfig& config);
    ~PatternDetector() = default;

    // Detect pattern and draw overlay
    bool detect(const cv::Mat& image,
                std::vector<cv::Point2f>& corners,
                cv::Mat& overlayImage);

    // Get detection confidence score (0.0 to 1.0)
    float getConfidenceScore() const { return confidenceScore_; }

    // Get expected number of corners
    int getExpectedCorners() const;

private:
    PatternConfig config_;
    float confidenceScore_;

    // ChArUco detector components
    cv::Ptr<cv::aruco::Dictionary> arucoDict_;
    cv::Ptr<cv::aruco::CharucoBoard> charucoBoard_;
    cv::Ptr<cv::aruco::DetectorParameters> detectorParams_;

    // Detection methods
    bool detectCheckerboard(const cv::Mat& image,
                          std::vector<cv::Point2f>& corners,
                          cv::Mat& overlayImage);

    bool detectCharuco(const cv::Mat& image,
                      std::vector<cv::Point2f>& corners,
                      cv::Mat& overlayImage);

    bool detectCircleGrid(const cv::Mat& image,
                         std::vector<cv::Point2f>& corners,
                         cv::Mat& overlayImage);

    // Helper methods
    void drawCheckerboardCorners(cv::Mat& image,
                                const std::vector<cv::Point2f>& corners);
    void drawCharucoCorners(cv::Mat& image,
                          const std::vector<cv::Point2f>& corners,
                          const std::vector<int>& ids);
};

} // namespace calibration
} // namespace unlook