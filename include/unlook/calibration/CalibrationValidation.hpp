/**
 * @file CalibrationValidation.hpp
 * @brief CRITICAL: Exhaustive calibration validation to prevent processing failures
 *
 * This module validates calibration data BEFORE any stereo processing.
 * If validation fails, processing MUST abort immediately.
 *
 * Based on MEGA_PROMPT_BACKEND_REWRITE.md specification.
 */

#ifndef UNLOOK_CALIBRATION_VALIDATION_HPP
#define UNLOOK_CALIBRATION_VALIDATION_HPP

#include <opencv2/core.hpp>
#include <string>
#include <vector>

namespace unlook {
namespace core {
    class Logger;
}
namespace calibration {
    class CalibrationManager;

/**
 * @brief CRITICAL: Validate calibration matches image resolution
 *
 * This structure contains all validation results and MUST be checked
 * before ANY stereo processing!
 */
struct CalibrationValidation {
    // ========== IMAGE RESOLUTION CONSISTENCY ==========
    cv::Size calibImageSize;       ///< From YAML: image_width × image_height
    cv::Size actualImageSize;      ///< From camera: actual captured size
    bool resolutionMatch;          ///< MUST be true!
    std::string resolutionError;

    // ========== RECTIFICATION MAPS CONSISTENCY ==========
    cv::Size map1LeftSize;         ///< map1Left.size()
    cv::Size map2LeftSize;         ///< map2Left.size()
    cv::Size map1RightSize;        ///< map1Right.size()
    cv::Size map2RightSize;        ///< map2Right.size()
    bool mapsConsistent;           ///< ALL maps must match calibImageSize
    std::string mapsError;

    // ========== CAMERA INTRINSICS VALIDATION ==========
    double fxLeft, fyLeft, cxLeft, cyLeft;
    double fxRight, fyRight, cxRight, cyRight;
    bool intrinsicsValid;          ///< fx,fy > 0, cx,cy within image bounds
    std::string intrinsicsError;

    // ========== STEREO EXTRINSICS VALIDATION ==========
    double baselineMM;             ///< From T vector, should be ~70mm
    bool baselineValid;            ///< 60mm < baseline < 80mm
    std::string baselineError;

    // ========== RECTIFICATION QUALITY ==========
    double cyLeftRect;             ///< From P1 projection matrix
    double cyRightRect;            ///< From P2 projection matrix
    double cyDifference;           ///< MUST be < 0.1 pixels (epipolar alignment!)
    bool epipolarAligned;
    std::string epipolarError;

    // ========== ROTATION MATRIX VALIDATION ==========
    bool rotationValid;            ///< det(R) ≈ 1.0, R*R^T ≈ I
    double rotationDeterminant;
    std::string rotationError;

    // ========== Q MATRIX VALIDATION ==========
    bool qMatrixValid;             ///< Q matrix exists and is 4x4
    std::string qMatrixError;

    // ========== OVERALL VALIDATION ==========
    bool allChecksPass;            ///< Master flag: ALL checks passed
    std::string errorMessage;      ///< Combined error message
    std::vector<std::string> warnings; ///< Non-critical warnings

    /**
     * @brief Validate calibration BEFORE any processing
     *
     * @param calib CalibrationManager containing loaded calibration
     * @param actualImageSize Actual image size from camera capture
     * @return CalibrationValidation structure with all validation results
     */
    static CalibrationValidation validate(
        const CalibrationManager& calib,
        const cv::Size& actualImageSize);

    /**
     * @brief Print detailed validation report to logger
     *
     * @param logger Logger instance for output
     */
    void printReport(core::Logger& logger) const;

    /**
     * @brief Get formatted validation report as string
     *
     * @return Multi-line string with all validation details
     */
    std::string getReportString() const;

private:
    /**
     * @brief Validate image resolution consistency
     */
    static void validateResolution(
        CalibrationValidation& result,
        const CalibrationManager& calib,
        const cv::Size& actualImageSize);

    /**
     * @brief Validate rectification maps consistency
     */
    static void validateRectificationMaps(
        CalibrationValidation& result,
        const CalibrationManager& calib);

    /**
     * @brief Validate camera intrinsics
     */
    static void validateIntrinsics(
        CalibrationValidation& result,
        const CalibrationManager& calib);

    /**
     * @brief Validate stereo extrinsics (baseline)
     */
    static void validateExtrinsics(
        CalibrationValidation& result,
        const CalibrationManager& calib);

    /**
     * @brief Validate epipolar alignment (cy values)
     */
    static void validateEpipolarAlignment(
        CalibrationValidation& result,
        const CalibrationManager& calib);

    /**
     * @brief Validate rotation matrix properties
     */
    static void validateRotationMatrix(
        CalibrationValidation& result,
        const CalibrationManager& calib);

    /**
     * @brief Validate Q matrix (disparity-to-depth)
     */
    static void validateQMatrix(
        CalibrationValidation& result,
        const CalibrationManager& calib);
};

} // namespace calibration
} // namespace unlook

#endif // UNLOOK_CALIBRATION_VALIDATION_HPP
