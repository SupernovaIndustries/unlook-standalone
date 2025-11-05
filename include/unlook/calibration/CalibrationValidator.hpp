#pragma once

#include <opencv2/core.hpp>
#include <vector>
#include <string>

namespace unlook {
namespace calibration {

// Forward declaration
struct CalibrationResult;

class CalibrationValidator {
public:
    struct ValidationCriteria {
        double maxRMSError;             // pixels (target for high precision)
        double maxEpipolarError;        // pixels
        double maxBaselineErrorMM;      // mm (for 70mm baseline)
        double maxBaselineErrorPercent; // percentage
        int minValidImagePairs;          // minimum for reliable calibration
        double minFocalLength;           // pixels (sanity check)
        double maxFocalLength;           // pixels (sanity check)
        double maxPrincipalPointOffset;  // pixels from image center

        // Constructor with defaults
        ValidationCriteria()
            : maxRMSError(0.3)
            , maxEpipolarError(0.5)
            , maxBaselineErrorMM(0.5)
            , maxBaselineErrorPercent(1.0)
            , minValidImagePairs(30)
            , minFocalLength(800.0)
            , maxFocalLength(2000.0)
            , maxPrincipalPointOffset(100.0)
        {}
    };

    CalibrationValidator() = default;
    ~CalibrationValidator() = default;

    // Main validation function
    bool validate(CalibrationResult& result,
                 const ValidationCriteria& criteria = ValidationCriteria());

    // Individual validation checks
    bool validateReprojectionError(const CalibrationResult& result,
                                  const ValidationCriteria& criteria);

    bool validateEpipolarError(const CalibrationResult& result,
                              const ValidationCriteria& criteria);

    bool validateBaseline(const CalibrationResult& result,
                         const ValidationCriteria& criteria);

    bool validateIntrinsics(const CalibrationResult& result,
                          const ValidationCriteria& criteria);

    bool validateImagePairs(const CalibrationResult& result,
                          const ValidationCriteria& criteria);

    // Compute epipolar error for a set of point correspondences
    double computeEpipolarError(const cv::Mat& fundamentalMatrix,
                               const std::vector<cv::Point2f>& pointsLeft,
                               const std::vector<cv::Point2f>& pointsRight);

    // Get validation messages
    std::vector<std::string> getWarnings() const { return warnings_; }
    std::vector<std::string> getErrors() const { return errors_; }
    std::vector<std::string> getInfo() const { return info_; }

    // Clear messages
    void clearMessages() {
        warnings_.clear();
        errors_.clear();
        info_.clear();
    }

private:
    std::vector<std::string> warnings_;
    std::vector<std::string> errors_;
    std::vector<std::string> info_;

    // Helper methods
    void addWarning(const std::string& msg);
    void addError(const std::string& msg);
    void addInfo(const std::string& msg);

    // Check matrix validity
    bool isMatrixValid(const cv::Mat& mat, const std::string& name);

    // Check if value is within reasonable range
    bool isValueReasonable(double value, double min, double max,
                         const std::string& name);
};

} // namespace calibration
} // namespace unlook