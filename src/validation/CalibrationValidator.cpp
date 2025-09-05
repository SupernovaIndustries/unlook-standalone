#include "unlook/validation/CalibrationValidator.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>

namespace unlook {
namespace validation {

// CalibrationValidator implementation
class CalibrationValidator::Impl {
public:
    std::shared_ptr<calibration::CalibrationManager> calibration;
    bool strictMode = true;
    bool verboseOutput = false;
    ValidationResult lastResult;
};

CalibrationValidator::CalibrationValidator() : pImpl(std::make_unique<Impl>()) {}
CalibrationValidator::~CalibrationValidator() = default;

bool CalibrationValidator::setCalibration(std::shared_ptr<calibration::CalibrationManager> calibration) {
    pImpl->calibration = calibration;
    return calibration && calibration->isCalibrationValid();
}

bool CalibrationValidator::validateComprehensive(ValidationResult& result) {
    if (!pImpl->calibration) return false;
    
    result.passed = true;
    result.overallScore = 100.0;
    
    // Get calibration data
    auto& calibData = pImpl->calibration->getCalibrationData();
    
    // Check RMS error
    result.rmsError = calibData.rmsError;
    if (result.rmsError > 0.2) {
        result.passed = false;
        result.overallScore -= 20;
        result.errors.push_back("RMS error exceeds 0.2 pixel threshold");
    } else if (result.rmsError > 0.15) {
        result.overallScore -= 10;
        result.warnings.push_back("RMS error approaching threshold");
    }
    
    // Check baseline
    result.baselineError = std::abs(calibData.baselineMm - 70.0);
    if (result.baselineError > 0.5) {
        result.overallScore -= 15;
        result.warnings.push_back("Baseline deviation exceeds 0.5mm");
    }
    
    // Store result
    pImpl->lastResult = result;
    
    return true;
}

bool CalibrationValidator::validateReprojectionError(
    const std::vector<std::vector<cv::Point2f>>& imagePoints,
    double maxAcceptableError) {
    // Placeholder implementation
    return true;
}

bool CalibrationValidator::validateEpipolarGeometry(
    const std::vector<cv::Point2f>& leftPoints,
    const std::vector<cv::Point2f>& rightPoints,
    EpipolarValidation& validation) {
    if (!pImpl->calibration || leftPoints.size() != rightPoints.size()) {
        return false;
    }
    
    // Compute epipolar lines
    pImpl->calibration->computeEpipolarLines(leftPoints, validation.leftEpilines, true);
    pImpl->calibration->computeEpipolarLines(rightPoints, validation.rightEpilines, false);
    
    // Compute distances
    validation.distances.clear();
    float sumDist = 0;
    float maxDist = 0;
    
    for (size_t i = 0; i < leftPoints.size(); ++i) {
        // Distance from point to epipolar line
        float a = validation.rightEpilines[i][0];
        float b = validation.rightEpilines[i][1];
        float c = validation.rightEpilines[i][2];
        
        float dist = std::abs(a * rightPoints[i].x + b * rightPoints[i].y + c) / 
                    std::sqrt(a * a + b * b);
        
        validation.distances.push_back(dist);
        sumDist += dist;
        maxDist = std::max(maxDist, dist);
    }
    
    validation.meanDistance = sumDist / leftPoints.size();
    validation.maxDistance = maxDist;
    
    // Compute standard deviation
    float sumSquares = 0;
    for (float dist : validation.distances) {
        float diff = dist - validation.meanDistance;
        sumSquares += diff * diff;
    }
    validation.stdDistance = std::sqrt(sumSquares / leftPoints.size());
    
    return validation.meanDistance < 1.0;  // Less than 1 pixel average error
}

double CalibrationValidator::validateRectification(const cv::Mat& leftRectified,
                                                  const cv::Mat& rightRectified,
                                                  double maxYDisparity) {
    // Placeholder implementation
    return 0.95;  // Return high score for now
}

bool CalibrationValidator::validateBaseline(double nominalBaselineMm,
                                           double toleranceMm) {
    if (!pImpl->calibration) return false;
    
    double actualBaseline = pImpl->calibration->getBaselineMm();
    double deviation = std::abs(actualBaseline - nominalBaselineMm);
    
    return deviation <= toleranceMm;
}

double CalibrationValidator::validateParameterConsistency() {
    if (!pImpl->calibration) return 0.0;
    
    auto& calibData = pImpl->calibration->getCalibrationData();
    double score = 1.0;
    
    // Check focal length consistency
    double fx_left = calibData.cameraMatrixLeft.at<double>(0, 0);
    double fy_left = calibData.cameraMatrixLeft.at<double>(1, 1);
    double fx_right = calibData.cameraMatrixRight.at<double>(0, 0);
    double fy_right = calibData.cameraMatrixRight.at<double>(1, 1);
    
    double focalDiff = std::abs(fx_left - fx_right) / std::max(fx_left, fx_right);
    if (focalDiff > 0.05) score -= 0.2;  // More than 5% difference
    
    // Check aspect ratio
    double aspectLeft = fx_left / fy_left;
    double aspectRight = fx_right / fy_right;
    double aspectDiff = std::abs(aspectLeft - aspectRight) / aspectLeft;
    if (aspectDiff > 0.02) score -= 0.1;  // More than 2% difference
    
    return std::max(0.0, score);
}

double CalibrationValidator::validateWithCheckerboard(const cv::Mat& leftImage,
                                                     const cv::Mat& rightImage,
                                                     const cv::Size& patternSize,
                                                     float squareSize) {
    // Placeholder implementation
    return 0.9;
}

bool CalibrationValidator::validateDepthAccuracy(const cv::Mat& depthMap,
                                                double targetDepthMm,
                                                const cv::Rect& roi,
                                                double toleranceMm) {
    if (depthMap.empty()) return false;
    
    cv::Mat roiDepth = depthMap(roi);
    cv::Scalar meanDepth = cv::mean(roiDepth, roiDepth > 0);
    
    double deviation = std::abs(meanDepth[0] - targetDepthMm);
    return deviation <= toleranceMm;
}

void CalibrationValidator::generateTestImages(cv::Mat& leftImage,
                                             cv::Mat& rightImage,
                                             const std::string& type) {
    // Generate test pattern images
    int width = 1456;
    int height = 1088;
    
    leftImage = cv::Mat::zeros(height, width, CV_8UC1);
    rightImage = cv::Mat::zeros(height, width, CV_8UC1);
    
    if (type == "checkerboard") {
        int squareSize = 50;
        for (int y = 0; y < height; y += squareSize) {
            for (int x = 0; x < width; x += squareSize) {
                bool white = ((x / squareSize) + (y / squareSize)) % 2 == 0;
                cv::Scalar color = white ? cv::Scalar(255) : cv::Scalar(0);
                cv::rectangle(leftImage, cv::Point(x, y), 
                            cv::Point(x + squareSize, y + squareSize), 
                            color, -1);
                cv::rectangle(rightImage, cv::Point(x, y), 
                            cv::Point(x + squareSize, y + squareSize), 
                            color, -1);
            }
        }
    }
}

void CalibrationValidator::visualizeEpipolarLines(const cv::Mat& leftImage,
                                                 const cv::Mat& rightImage,
                                                 const std::vector<cv::Point2f>& points,
                                                 cv::Mat& visualization) {
    // Placeholder implementation
    cv::hconcat(leftImage, rightImage, visualization);
}

void CalibrationValidator::visualizeRectification(const cv::Mat& leftRectified,
                                                 const cv::Mat& rightRectified,
                                                 cv::Mat& visualization) {
    // Draw horizontal lines to show alignment
    cv::Mat combined;
    cv::hconcat(leftRectified, rightRectified, combined);
    
    if (combined.channels() == 1) {
        cv::cvtColor(combined, visualization, cv::COLOR_GRAY2BGR);
    } else {
        visualization = combined.clone();
    }
    
    // Draw horizontal lines
    for (int y = 0; y < visualization.rows; y += 50) {
        cv::line(visualization, cv::Point(0, y), 
                cv::Point(visualization.cols, y), 
                cv::Scalar(0, 255, 0), 1);
    }
}

void CalibrationValidator::setValidationParameters(bool strictMode,
                                                  bool verboseOutput) {
    pImpl->strictMode = strictMode;
    pImpl->verboseOutput = verboseOutput;
}

std::string CalibrationValidator::getDetailedReport() const {
    return pImpl->lastResult.generateReport();
}

bool CalibrationValidator::exportResults(const std::string& filename,
                                        const std::string& format) const {
    // Placeholder implementation
    return false;
}

double CalibrationValidator::computeExpectedPrecision(double depthMm) const {
    if (!pImpl->calibration) return 0.0;
    
    double baseline = pImpl->calibration->getBaselineMm();
    double focal = 1700;  // Approximate focal length
    double disparityError = 0.2;  // Sub-pixel error
    
    double precision = (depthMm * depthMm * disparityError) / (baseline * focal);
    return precision;
}

bool CalibrationValidator::validateIndustrialPrecision() {
    if (!pImpl->calibration) return false;
    
    // Check all industrial requirements
    bool meetsRequirements = true;
    
    // RMS error must be < 0.2 pixels
    if (pImpl->calibration->getRmsError() >= 0.2) {
        meetsRequirements = false;
    }
    
    // Baseline must be within 0.1mm of nominal
    if (std::abs(pImpl->calibration->getBaselineMm() - 70.0) > 0.1) {
        meetsRequirements = false;
    }
    
    // Expected precision at 100mm must be < 0.005mm
    double precisionAt100mm = computeExpectedPrecision(100.0);
    if (precisionAt100mm > 0.005) {
        meetsRequirements = false;
    }
    
    return meetsRequirements;
}

// ValidationResult implementation
std::string ValidationResult::generateReport() const {
    std::stringstream ss;
    ss << "Validation Report\n";
    ss << "================\n";
    ss << "Overall Status: " << (passed ? "PASSED" : "FAILED") << "\n";
    ss << "Overall Score: " << overallScore << "/100\n\n";
    
    ss << "Metrics:\n";
    ss << "  RMS Error: " << rmsError << " pixels\n";
    ss << "  Max Error: " << maxError << " pixels\n";
    ss << "  Mean Error: " << meanError << " pixels\n";
    ss << "  Epipolar Error: " << epipolarError << " pixels\n";
    ss << "  Baseline Error: " << baselineError << " mm\n\n";
    
    if (!warnings.empty()) {
        ss << "Warnings:\n";
        for (const auto& w : warnings) {
            ss << "  - " << w << "\n";
        }
        ss << "\n";
    }
    
    if (!errors.empty()) {
        ss << "Errors:\n";
        for (const auto& e : errors) {
            ss << "  ! " << e << "\n";
        }
    }
    
    return ss.str();
}

} // namespace validation
} // namespace unlook