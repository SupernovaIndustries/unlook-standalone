#include "unlook/calibration/CalibrationValidator.hpp"
#include "unlook/calibration/StereoCalibrationProcessor.hpp"
#include "unlook/core/Logger.hpp"
#include <opencv2/calib3d.hpp>
#include <cmath>
#include <sstream>
#include <iomanip>

namespace unlook {
namespace calibration {

bool CalibrationValidator::validate(CalibrationResult& result,
                                   const ValidationCriteria& criteria) {
    clearMessages();

    bool allPassed = true;

    // 1. Validate image pairs
    if (!validateImagePairs(result, criteria)) {
        allPassed = false;
    }

    // 2. Validate reprojection error
    if (!validateReprojectionError(result, criteria)) {
        allPassed = false;
    }

    // 3. Validate epipolar error
    if (!validateEpipolarError(result, criteria)) {
        allPassed = false;
    }

    // 4. Validate baseline
    if (!validateBaseline(result, criteria)) {
        allPassed = false;
    }

    // 5. Validate intrinsics
    if (!validateIntrinsics(result, criteria)) {
        allPassed = false;
    }

    // Update result with validation messages
    result.warnings = warnings_;
    result.errors = errors_;
    result.qualityPassed = allPassed && errors_.empty();

    // Set check statuses
    if (result.rmsReprojectionError <= criteria.maxRMSError) {
        result.rmsCheck = "PASS";
    } else if (result.rmsReprojectionError <= criteria.maxRMSError * 1.5) {
        result.rmsCheck = "WARNING";
    } else {
        result.rmsCheck = "FAIL";
    }

    if (result.baselineErrorMM <= criteria.maxBaselineErrorMM) {
        result.baselineCheck = "PASS";
    } else if (result.baselineErrorMM <= criteria.maxBaselineErrorMM * 2) {
        result.baselineCheck = "WARNING";
    } else {
        result.baselineCheck = "FAIL";
    }

    if (result.meanEpipolarError <= criteria.maxEpipolarError) {
        result.epipolarCheck = "PASS";
    } else if (result.meanEpipolarError <= criteria.maxEpipolarError * 1.5) {
        result.epipolarCheck = "WARNING";
    } else {
        result.epipolarCheck = "FAIL";
    }

    // Log validation summary
    core::Logger::getInstance().info("Calibration validation complete:");
    core::Logger::getInstance().info("  - Quality: " +
                                    (result.qualityPassed ? std::string("PASSED") : std::string("FAILED")));
    core::Logger::getInstance().info("  - RMS check: " + result.rmsCheck);
    core::Logger::getInstance().info("  - Baseline check: " + result.baselineCheck);
    core::Logger::getInstance().info("  - Epipolar check: " + result.epipolarCheck);

    if (!warnings_.empty()) {
        core::Logger::getInstance().warning("Validation warnings: " + std::to_string(warnings_.size()));
        for (const auto& warning : warnings_) {
            core::Logger::getInstance().warning("  - " + warning);
        }
    }

    if (!errors_.empty()) {
        core::Logger::getInstance().error("Validation errors: " + std::to_string(errors_.size()));
        for (const auto& error : errors_) {
            core::Logger::getInstance().error("  - " + error);
        }
    }

    return allPassed;
}

bool CalibrationValidator::validateReprojectionError(const CalibrationResult& result,
                                                    const ValidationCriteria& criteria) {
    bool passed = true;
    std::stringstream ss;

    // Check RMS error
    if (result.rmsReprojectionError > criteria.maxRMSError * 2) {
        ss.str("");
        ss << "RMS reprojection error " << std::fixed << std::setprecision(3)
           << result.rmsReprojectionError << " pixels EXCEEDS limit (max: "
           << criteria.maxRMSError << " pixels)";
        addError(ss.str());
        passed = false;
    } else if (result.rmsReprojectionError > criteria.maxRMSError) {
        ss.str("");
        ss << "RMS reprojection error " << std::fixed << std::setprecision(3)
           << result.rmsReprojectionError << " pixels above target (target: "
           << criteria.maxRMSError << " pixels)";
        addWarning(ss.str());
    } else {
        ss.str("");
        ss << "RMS reprojection error " << std::fixed << std::setprecision(3)
           << result.rmsReprojectionError << " pixels (GOOD)";
        addInfo(ss.str());
    }

    // Check left/right balance
    double imbalance = std::abs(result.meanReprojectionErrorLeft -
                               result.meanReprojectionErrorRight);
    if (imbalance > 0.1) {
        ss.str("");
        ss << "Reprojection error imbalance between cameras: Left="
           << std::fixed << std::setprecision(3) << result.meanReprojectionErrorLeft
           << " Right=" << result.meanReprojectionErrorRight;
        addWarning(ss.str());
    }

    // Check maximum error
    if (result.maxReprojectionError > 1.0) {
        ss.str("");
        ss << "Maximum reprojection error " << std::fixed << std::setprecision(3)
           << result.maxReprojectionError << " pixels is high (some outliers present)";
        addWarning(ss.str());
    }

    return passed;
}

bool CalibrationValidator::validateEpipolarError(const CalibrationResult& result,
                                                const ValidationCriteria& criteria) {
    bool passed = true;
    std::stringstream ss;

    // Check mean epipolar error
    if (result.meanEpipolarError > criteria.maxEpipolarError * 2) {
        ss.str("");
        ss << "Mean epipolar error " << std::fixed << std::setprecision(3)
           << result.meanEpipolarError << " pixels EXCEEDS limit (max: "
           << criteria.maxEpipolarError << " pixels)";
        addError(ss.str());
        passed = false;
    } else if (result.meanEpipolarError > criteria.maxEpipolarError) {
        ss.str("");
        ss << "Mean epipolar error " << std::fixed << std::setprecision(3)
           << result.meanEpipolarError << " pixels above target (target: "
           << criteria.maxEpipolarError << " pixels)";
        addWarning(ss.str());
    } else {
        ss.str("");
        ss << "Mean epipolar error " << std::fixed << std::setprecision(3)
           << result.meanEpipolarError << " pixels (GOOD)";
        addInfo(ss.str());
    }

    // Check maximum epipolar error
    if (result.maxEpipolarError > 2.0) {
        ss.str("");
        ss << "Maximum epipolar error " << std::fixed << std::setprecision(3)
           << result.maxEpipolarError << " pixels indicates rectification issues";
        addWarning(ss.str());
    }

    return passed;
}

bool CalibrationValidator::validateBaseline(const CalibrationResult& result,
                                           const ValidationCriteria& criteria) {
    bool passed = true;
    std::stringstream ss;

    // Check baseline error in mm
    if (result.baselineErrorMM > criteria.maxBaselineErrorMM * 2) {
        ss.str("");
        ss << "Baseline error " << std::fixed << std::setprecision(2)
           << result.baselineErrorMM << "mm CRITICAL (max: "
           << criteria.maxBaselineErrorMM << "mm)";
        addError(ss.str());
        passed = false;
    } else if (result.baselineErrorMM > criteria.maxBaselineErrorMM) {
        ss.str("");
        ss << "Baseline error " << std::fixed << std::setprecision(2)
           << result.baselineErrorMM << "mm above tolerance (max: "
           << criteria.maxBaselineErrorMM << "mm)";
        addWarning(ss.str());
    } else {
        ss.str("");
        ss << "Baseline " << std::fixed << std::setprecision(2)
           << result.baselineMM << "mm (error: " << result.baselineErrorMM << "mm GOOD)";
        addInfo(ss.str());
    }

    // Check baseline error percentage
    if (result.baselineErrorPercent > criteria.maxBaselineErrorPercent) {
        ss.str("");
        ss << "Baseline error " << std::fixed << std::setprecision(2)
           << result.baselineErrorPercent << "% exceeds tolerance";
        if (result.baselineErrorPercent > criteria.maxBaselineErrorPercent * 2) {
            addError(ss.str());
            passed = false;
        } else {
            addWarning(ss.str());
        }
    }

    // Sanity check on baseline value
    if (result.baselineMM < 10.0 || result.baselineMM > 500.0) {
        ss.str("");
        ss << "Baseline value " << std::fixed << std::setprecision(2)
           << result.baselineMM << "mm is unrealistic for stereo system";
        addError(ss.str());
        passed = false;
    }

    return passed;
}

bool CalibrationValidator::validateIntrinsics(const CalibrationResult& result,
                                             const ValidationCriteria& criteria) {
    bool passed = true;
    std::stringstream ss;

    // Check camera matrix validity
    if (!isMatrixValid(result.cameraMatrixLeft, "Left camera matrix") ||
        !isMatrixValid(result.cameraMatrixRight, "Right camera matrix")) {
        passed = false;
    }

    // Extract focal lengths
    double fxLeft = result.cameraMatrixLeft.at<double>(0, 0);
    double fyLeft = result.cameraMatrixLeft.at<double>(1, 1);
    double fxRight = result.cameraMatrixRight.at<double>(0, 0);
    double fyRight = result.cameraMatrixRight.at<double>(1, 1);

    // Check focal length range
    if (!isValueReasonable(fxLeft, criteria.minFocalLength, criteria.maxFocalLength, "Left fx") ||
        !isValueReasonable(fyLeft, criteria.minFocalLength, criteria.maxFocalLength, "Left fy") ||
        !isValueReasonable(fxRight, criteria.minFocalLength, criteria.maxFocalLength, "Right fx") ||
        !isValueReasonable(fyRight, criteria.minFocalLength, criteria.maxFocalLength, "Right fy")) {
        passed = false;
    }

    // Check focal length aspect ratio
    double aspectLeft = fxLeft / fyLeft;
    double aspectRight = fxRight / fyRight;

    if (std::abs(aspectLeft - 1.0) > 0.1) {
        ss.str("");
        ss << "Left camera aspect ratio " << std::fixed << std::setprecision(3)
           << aspectLeft << " deviates from expected 1.0";
        addWarning(ss.str());
    }

    if (std::abs(aspectRight - 1.0) > 0.1) {
        ss.str("");
        ss << "Right camera aspect ratio " << std::fixed << std::setprecision(3)
           << aspectRight << " deviates from expected 1.0";
        addWarning(ss.str());
    }

    // Check principal points
    double cxLeft = result.cameraMatrixLeft.at<double>(0, 2);
    double cyLeft = result.cameraMatrixLeft.at<double>(1, 2);
    double cxRight = result.cameraMatrixRight.at<double>(0, 2);
    double cyRight = result.cameraMatrixRight.at<double>(1, 2);

    double imageCenterX = result.imageSize.width / 2.0;
    double imageCenterY = result.imageSize.height / 2.0;

    auto checkPrincipalPoint = [&](double cx, double cy, const std::string& camera) {
        double offsetX = std::abs(cx - imageCenterX);
        double offsetY = std::abs(cy - imageCenterY);

        if (offsetX > criteria.maxPrincipalPointOffset ||
            offsetY > criteria.maxPrincipalPointOffset) {
            ss.str("");
            ss << camera << " principal point (" << std::fixed << std::setprecision(1)
               << cx << ", " << cy << ") far from image center ("
               << imageCenterX << ", " << imageCenterY << ")";
            addWarning(ss.str());
        }
    };

    checkPrincipalPoint(cxLeft, cyLeft, "Left");
    checkPrincipalPoint(cxRight, cyRight, "Right");

    // Check distortion coefficients
    if (!isMatrixValid(result.distCoeffsLeft, "Left distortion") ||
        !isMatrixValid(result.distCoeffsRight, "Right distortion")) {
        passed = false;
    }

    // Check for extreme distortion
    auto checkDistortion = [&](const cv::Mat& dist, const std::string& camera) {
        if (dist.rows >= 2) {
            double k1 = dist.at<double>(0, 0);
            double k2 = dist.at<double>(1, 0);

            if (std::abs(k1) > 1.0 || std::abs(k2) > 1.0) {
                ss.str("");
                ss << camera << " has extreme distortion coefficients (k1="
                   << std::fixed << std::setprecision(3) << k1
                   << ", k2=" << k2 << ")";
                addWarning(ss.str());
            }
        }
    };

    checkDistortion(result.distCoeffsLeft, "Left camera");
    checkDistortion(result.distCoeffsRight, "Right camera");

    return passed;
}

bool CalibrationValidator::validateImagePairs(const CalibrationResult& result,
                                             const ValidationCriteria& criteria) {
    bool passed = true;
    std::stringstream ss;

    if (result.validImagePairs < criteria.minValidImagePairs) {
        ss.str("");
        ss << "Insufficient valid image pairs: " << result.validImagePairs
           << " (minimum required: " << criteria.minValidImagePairs << ")";
        addError(ss.str());
        passed = false;
    } else {
        ss.str("");
        ss << "Valid image pairs: " << result.validImagePairs
           << "/" << result.numImagePairs;
        addInfo(ss.str());
    }

    // Check success rate
    double successRate = static_cast<double>(result.validImagePairs) /
                        static_cast<double>(result.numImagePairs);

    if (successRate < 0.6) {
        ss.str("");
        ss << "Low pattern detection rate: " << std::fixed << std::setprecision(1)
           << (successRate * 100) << "%";
        addWarning(ss.str());
    }

    return passed;
}

double CalibrationValidator::computeEpipolarError(const cv::Mat& fundamentalMatrix,
                                                 const std::vector<cv::Point2f>& pointsLeft,
                                                 const std::vector<cv::Point2f>& pointsRight) {
    if (pointsLeft.size() != pointsRight.size() || pointsLeft.empty()) {
        return -1.0;
    }

    double totalError = 0.0;
    int count = 0;

    for (size_t i = 0; i < pointsLeft.size(); ++i) {
        // Convert points to homogeneous coordinates
        cv::Mat pt1 = (cv::Mat_<double>(3, 1) << pointsLeft[i].x, pointsLeft[i].y, 1.0);
        cv::Mat pt2 = (cv::Mat_<double>(3, 1) << pointsRight[i].x, pointsRight[i].y, 1.0);

        // Compute epipolar line in second image: l2 = F * p1
        cv::Mat epiline2 = fundamentalMatrix * pt1;

        // Compute distance from point to epipolar line
        double a = epiline2.at<double>(0, 0);
        double b = epiline2.at<double>(1, 0);
        double c = epiline2.at<double>(2, 0);

        double distance = std::abs(a * pointsRight[i].x + b * pointsRight[i].y + c) /
                         std::sqrt(a * a + b * b);

        totalError += distance;

        // Also check reverse direction
        cv::Mat epiline1 = fundamentalMatrix.t() * pt2;
        a = epiline1.at<double>(0, 0);
        b = epiline1.at<double>(1, 0);
        c = epiline1.at<double>(2, 0);

        distance = std::abs(a * pointsLeft[i].x + b * pointsLeft[i].y + c) /
                  std::sqrt(a * a + b * b);

        totalError += distance;
        count += 2;
    }

    return count > 0 ? totalError / count : 0.0;
}

void CalibrationValidator::addWarning(const std::string& msg) {
    warnings_.push_back(msg);
    core::Logger::getInstance().warning("Validation: " + msg);
}

void CalibrationValidator::addError(const std::string& msg) {
    errors_.push_back(msg);
    core::Logger::getInstance().error("Validation: " + msg);
}

void CalibrationValidator::addInfo(const std::string& msg) {
    info_.push_back(msg);
    core::Logger::getInstance().info("Validation: " + msg);
}

bool CalibrationValidator::isMatrixValid(const cv::Mat& mat, const std::string& name) {
    if (mat.empty()) {
        addError(name + " is empty");
        return false;
    }

    // Check for NaN or Inf
    for (int i = 0; i < mat.rows; ++i) {
        for (int j = 0; j < mat.cols; ++j) {
            double val = mat.at<double>(i, j);
            if (std::isnan(val) || std::isinf(val)) {
                addError(name + " contains NaN or Inf values");
                return false;
            }
        }
    }

    return true;
}

bool CalibrationValidator::isValueReasonable(double value, double min, double max,
                                            const std::string& name) {
    if (value < min || value > max) {
        std::stringstream ss;
        ss << name << " value " << std::fixed << std::setprecision(2) << value
           << " outside reasonable range [" << min << ", " << max << "]";
        addError(ss.str());
        return false;
    }
    return true;
}

} // namespace calibration
} // namespace unlook