/**
 * @file CalibrationValidation.cpp
 * @brief Implementation of exhaustive calibration validation
 *
 * Based on MEGA_PROMPT_BACKEND_REWRITE.md specification.
 */

#include "unlook/calibration/CalibrationValidation.hpp"
#include "unlook/calibration/CalibrationManager.hpp"
#include "unlook/core/Logger.hpp"
#include <opencv2/core.hpp>
#include <sstream>
#include <iomanip>
#include <cmath>

namespace unlook {
namespace calibration {

// ========== MAIN VALIDATION ENTRY POINT ==========

CalibrationValidation CalibrationValidation::validate(
    const CalibrationManager& calib,
    const cv::Size& actualImageSize)
{
    CalibrationValidation result;
    result.actualImageSize = actualImageSize;
    result.allChecksPass = true; // Assume true, set to false if any check fails

    // Perform all validation checks
    validateResolution(result, calib, actualImageSize);
    validateRectificationMaps(result, calib);
    validateIntrinsics(result, calib);
    validateExtrinsics(result, calib);
    validateEpipolarAlignment(result, calib);
    validateRotationMatrix(result, calib);
    validateQMatrix(result, calib);

    // Build combined error message
    std::ostringstream oss;
    if (!result.resolutionMatch) {
        oss << "Resolution mismatch! " << result.resolutionError << "\n";
        result.allChecksPass = false;
    }
    if (!result.mapsConsistent) {
        oss << "Rectification maps inconsistent! " << result.mapsError << "\n";
        result.allChecksPass = false;
    }
    if (!result.intrinsicsValid) {
        oss << "Camera intrinsics invalid! " << result.intrinsicsError << "\n";
        result.allChecksPass = false;
    }
    if (!result.baselineValid) {
        oss << "Baseline out of range! " << result.baselineError << "\n";
        result.allChecksPass = false;
    }
    if (!result.epipolarAligned) {
        oss << "Epipolar alignment failed! " << result.epipolarError << "\n";
        result.allChecksPass = false;
    }
    if (!result.rotationValid) {
        oss << "Rotation matrix invalid! " << result.rotationError << "\n";
        result.allChecksPass = false;
    }
    if (!result.qMatrixValid) {
        oss << "Q matrix invalid! " << result.qMatrixError << "\n";
        result.allChecksPass = false;
    }

    result.errorMessage = oss.str();
    return result;
}

// ========== RESOLUTION VALIDATION ==========

void CalibrationValidation::validateResolution(
    CalibrationValidation& result,
    const CalibrationManager& calib,
    const cv::Size& actualImageSize)
{
    const auto& data = calib.getCalibrationData();
    result.calibImageSize = data.imageSize;

    if (result.calibImageSize.width == actualImageSize.width &&
        result.calibImageSize.height == actualImageSize.height) {
        result.resolutionMatch = true;
        result.resolutionError = "";
    } else {
        result.resolutionMatch = false;
        std::ostringstream oss;
        oss << "Calibration is for " << result.calibImageSize.width << "x"
            << result.calibImageSize.height << " but actual image is "
            << actualImageSize.width << "x" << actualImageSize.height
            << ". Rectification will fail!";
        result.resolutionError = oss.str();
    }
}

// ========== RECTIFICATION MAPS VALIDATION ==========

void CalibrationValidation::validateRectificationMaps(
    CalibrationValidation& result,
    const CalibrationManager& calib)
{
    const auto& data = calib.getCalibrationData();

    result.map1LeftSize = data.map1Left.size();
    result.map2LeftSize = data.map2Left.size();
    result.map1RightSize = data.map1Right.size();
    result.map2RightSize = data.map2Right.size();

    result.mapsConsistent = true;
    std::ostringstream oss;

    // Check if maps are computed
    if (data.map1Left.empty() || data.map2Left.empty() ||
        data.map1Right.empty() || data.map2Right.empty()) {
        result.warnings.push_back("Rectification maps not yet computed (will be computed on first use)");
        result.mapsConsistent = true; // Not an error, just a warning
        return;
    }

    // Check map sizes match calibration size
    if (result.map1LeftSize != result.calibImageSize) {
        oss << "map1Left size " << result.map1LeftSize << " != calibration size "
            << result.calibImageSize << "; ";
        result.mapsConsistent = false;
    }
    if (result.map2LeftSize != result.calibImageSize) {
        oss << "map2Left size " << result.map2LeftSize << " != calibration size "
            << result.calibImageSize << "; ";
        result.mapsConsistent = false;
    }
    if (result.map1RightSize != result.calibImageSize) {
        oss << "map1Right size " << result.map1RightSize << " != calibration size "
            << result.calibImageSize << "; ";
        result.mapsConsistent = false;
    }
    if (result.map2RightSize != result.calibImageSize) {
        oss << "map2Right size " << result.map2RightSize << " != calibration size "
            << result.calibImageSize << "; ";
        result.mapsConsistent = false;
    }

    result.mapsError = oss.str();
}

// ========== CAMERA INTRINSICS VALIDATION ==========

void CalibrationValidation::validateIntrinsics(
    CalibrationValidation& result,
    const CalibrationManager& calib)
{
    const auto& data = calib.getCalibrationData();

    // Extract intrinsics from camera matrices
    result.fxLeft = data.cameraMatrixLeft.at<double>(0, 0);
    result.fyLeft = data.cameraMatrixLeft.at<double>(1, 1);
    result.cxLeft = data.cameraMatrixLeft.at<double>(0, 2);
    result.cyLeft = data.cameraMatrixLeft.at<double>(1, 2);

    result.fxRight = data.cameraMatrixRight.at<double>(0, 0);
    result.fyRight = data.cameraMatrixRight.at<double>(1, 1);
    result.cxRight = data.cameraMatrixRight.at<double>(0, 2);
    result.cyRight = data.cameraMatrixRight.at<double>(1, 2);

    result.intrinsicsValid = true;
    std::ostringstream oss;

    // Validate focal lengths (must be positive)
    if (result.fxLeft <= 0 || result.fyLeft <= 0) {
        oss << "Left camera focal lengths invalid (fx=" << result.fxLeft
            << ", fy=" << result.fyLeft << "); ";
        result.intrinsicsValid = false;
    }
    if (result.fxRight <= 0 || result.fyRight <= 0) {
        oss << "Right camera focal lengths invalid (fx=" << result.fxRight
            << ", fy=" << result.fyRight << "); ";
        result.intrinsicsValid = false;
    }

    // Validate principal points (must be within image bounds)
    if (result.cxLeft < 0 || result.cxLeft > result.calibImageSize.width ||
        result.cyLeft < 0 || result.cyLeft > result.calibImageSize.height) {
        oss << "Left principal point out of bounds (cx=" << result.cxLeft
            << ", cy=" << result.cyLeft << "); ";
        result.intrinsicsValid = false;
    }
    if (result.cxRight < 0 || result.cxRight > result.calibImageSize.width ||
        result.cyRight < 0 || result.cyRight > result.calibImageSize.height) {
        oss << "Right principal point out of bounds (cx=" << result.cxRight
            << ", cy=" << result.cyRight << "); ";
        result.intrinsicsValid = false;
    }

    result.intrinsicsError = oss.str();
}

// ========== STEREO EXTRINSICS VALIDATION ==========

void CalibrationValidation::validateExtrinsics(
    CalibrationValidation& result,
    const CalibrationManager& calib)
{
    const auto& data = calib.getCalibrationData();

    // Calculate baseline from translation vector (T[0] is baseline in mm)
    result.baselineMM = std::abs(data.T.at<double>(0, 0));

    // Baseline should be approximately 70mm (valid range: 60-80mm)
    const double MIN_BASELINE_MM = 60.0;
    const double MAX_BASELINE_MM = 80.0;

    if (result.baselineMM >= MIN_BASELINE_MM &&
        result.baselineMM <= MAX_BASELINE_MM) {
        result.baselineValid = true;
        result.baselineError = "";
    } else {
        result.baselineValid = false;
        std::ostringstream oss;
        oss << "Baseline " << std::fixed << std::setprecision(2)
            << result.baselineMM << "mm is outside valid range ["
            << MIN_BASELINE_MM << ", " << MAX_BASELINE_MM << "]mm";
        result.baselineError = oss.str();
    }
}

// ========== EPIPOLAR ALIGNMENT VALIDATION ==========

void CalibrationValidation::validateEpipolarAlignment(
    CalibrationValidation& result,
    const CalibrationManager& calib)
{
    const auto& data = calib.getCalibrationData();

    // Extract cy from projection matrices P1 and P2
    // P1 and P2 are 3x4 matrices: cy is at position [1,2]
    result.cyLeftRect = data.P1.at<double>(1, 2);
    result.cyRightRect = data.P2.at<double>(1, 2);
    result.cyDifference = std::abs(result.cyLeftRect - result.cyRightRect);

    // cy values MUST be nearly identical for horizontal epipolar lines
    const double MAX_CY_DIFF = 0.1; // pixels

    if (result.cyDifference < MAX_CY_DIFF) {
        result.epipolarAligned = true;
        result.epipolarError = "";
    } else {
        result.epipolarAligned = false;
        std::ostringstream oss;
        oss << "Epipolar lines NOT horizontal! cy_left=" << std::fixed
            << std::setprecision(3) << result.cyLeftRect << ", cy_right="
            << result.cyRightRect << ", difference=" << result.cyDifference
            << " pixels (max allowed: " << MAX_CY_DIFF << ")";
        result.epipolarError = oss.str();
    }
}

// ========== ROTATION MATRIX VALIDATION ==========

void CalibrationValidation::validateRotationMatrix(
    CalibrationValidation& result,
    const CalibrationManager& calib)
{
    const auto& data = calib.getCalibrationData();

    // A valid rotation matrix R must satisfy:
    // 1. det(R) = 1 (or very close to 1)
    // 2. R * R^T = I (orthonormal)

    result.rotationDeterminant = cv::determinant(data.R);

    result.rotationValid = true;
    std::ostringstream oss;

    // Check determinant is close to 1.0
    const double DET_TOLERANCE = 0.01;
    if (std::abs(result.rotationDeterminant - 1.0) > DET_TOLERANCE) {
        oss << "det(R) = " << std::fixed << std::setprecision(6)
            << result.rotationDeterminant << " (expected ~1.0); ";
        result.rotationValid = false;
    }

    // Check orthonormality: R * R^T should be identity
    cv::Mat RRt = data.R * data.R.t();
    cv::Mat identity = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat diff = RRt - identity;
    double maxError = cv::norm(diff, cv::NORM_INF);

    const double ORTHO_TOLERANCE = 0.01;
    if (maxError > ORTHO_TOLERANCE) {
        oss << "R*R^T != I (max error = " << std::fixed << std::setprecision(6)
            << maxError << "); ";
        result.rotationValid = false;
    }

    result.rotationError = oss.str();
}

// ========== Q MATRIX VALIDATION ==========

void CalibrationValidation::validateQMatrix(
    CalibrationValidation& result,
    const CalibrationManager& calib)
{
    const auto& data = calib.getCalibrationData();

    result.qMatrixValid = true;
    std::ostringstream oss;

    // Check Q matrix exists and has correct size
    if (data.Q.empty()) {
        oss << "Q matrix is empty; ";
        result.qMatrixValid = false;
    } else if (data.Q.rows != 4 || data.Q.cols != 4) {
        oss << "Q matrix size is " << data.Q.rows << "x" << data.Q.cols
            << " (expected 4x4); ";
        result.qMatrixValid = false;
    } else if (data.Q.type() != CV_64F && data.Q.type() != CV_32F) {
        oss << "Q matrix type is not floating point; ";
        result.qMatrixValid = false;
    }

    result.qMatrixError = oss.str();
}

// ========== REPORT GENERATION ==========

void CalibrationValidation::printReport(core::Logger& logger) const {
    logger.info("╔════════════════════════════════════════════════════════════════╗");
    logger.info("║          CALIBRATION VALIDATION REPORT                         ║");
    logger.info("╚════════════════════════════════════════════════════════════════╝");

    // Resolution
    logger.info("Image Resolution:");
    logger.info("  Calibration: " + std::to_string(calibImageSize.width) + "x" +
                std::to_string(calibImageSize.height));
    logger.info("  Actual:      " + std::to_string(actualImageSize.width) + "x" +
                std::to_string(actualImageSize.height));
    logger.info("  Match: " + std::string(resolutionMatch ? "✓ YES" : "✗ NO"));

    // Rectification maps
    if (!map1LeftSize.empty()) {
        logger.info("Rectification Maps:");
        logger.info("  map1Left:  " + std::to_string(map1LeftSize.width) + "x" +
                    std::to_string(map1LeftSize.height));
        logger.info("  map1Right: " + std::to_string(map1RightSize.width) + "x" +
                    std::to_string(map1RightSize.height));
        logger.info("  Consistent: " + std::string(mapsConsistent ? "✓ YES" : "✗ NO"));
    }

    // Intrinsics
    logger.info("Camera Intrinsics:");
    std::ostringstream ossLeft, ossRight;
    ossLeft << std::fixed << std::setprecision(1)
            << "  LEFT:  fx=" << fxLeft << ", fy=" << fyLeft
            << ", cx=" << cxLeft << ", cy=" << cyLeft;
    ossRight << std::fixed << std::setprecision(1)
             << "  RIGHT: fx=" << fxRight << ", fy=" << fyRight
             << ", cx=" << cxRight << ", cy=" << cyRight;
    logger.info(ossLeft.str());
    logger.info(ossRight.str());
    logger.info("  Valid: " + std::string(intrinsicsValid ? "✓ YES" : "✗ NO"));

    // Extrinsics
    logger.info("Stereo Extrinsics:");
    std::ostringstream ossBaseline;
    ossBaseline << std::fixed << std::setprecision(2)
                << "  Baseline: " << baselineMM << " mm";
    logger.info(ossBaseline.str());
    logger.info("  Valid range [60-80mm]: " +
                std::string(baselineValid ? "✓ YES" : "✗ NO"));

    // Epipolar alignment
    logger.info("Epipolar Alignment:");
    std::ostringstream ossEpipolar;
    ossEpipolar << std::fixed << std::setprecision(3)
                << "  cy_left:  " << cyLeftRect << " px"
                << "\n  cy_right: " << cyRightRect << " px"
                << "\n  Difference: " << cyDifference << " px";
    logger.info(ossEpipolar.str());
    logger.info("  Aligned (<0.1px): " +
                std::string(epipolarAligned ? "✓ YES" : "✗ NO"));

    // Rotation matrix
    logger.info("Rotation Matrix:");
    std::ostringstream ossRot;
    ossRot << std::fixed << std::setprecision(6)
           << "  det(R): " << rotationDeterminant;
    logger.info(ossRot.str());
    logger.info("  Valid: " + std::string(rotationValid ? "✓ YES" : "✗ NO"));

    // Q matrix
    logger.info("Q Matrix:");
    logger.info("  Valid: " + std::string(qMatrixValid ? "✓ YES" : "✗ NO"));

    // Warnings
    if (!warnings.empty()) {
        logger.info("Warnings:");
        for (const auto& warning : warnings) {
            logger.info("  ⚠ " + warning);
        }
    }

    // Overall result
    logger.info("─────────────────────────────────────────────────────────────────");
    if (allChecksPass) {
        logger.info("✓✓✓ ALL VALIDATION CHECKS PASSED ✓✓✓");
    } else {
        logger.error("✗✗✗ VALIDATION FAILED ✗✗✗");
        logger.error("Errors:");
        logger.error(errorMessage);
    }
    logger.info("═════════════════════════════════════════════════════════════════");
}

std::string CalibrationValidation::getReportString() const {
    std::ostringstream oss;
    oss << "╔════════════════════════════════════════════════════════════════╗\n";
    oss << "║          CALIBRATION VALIDATION REPORT                         ║\n";
    oss << "╚════════════════════════════════════════════════════════════════╝\n";

    oss << "Image Resolution:\n";
    oss << "  Calibration: " << calibImageSize.width << "x" << calibImageSize.height << "\n";
    oss << "  Actual:      " << actualImageSize.width << "x" << actualImageSize.height << "\n";
    oss << "  Match: " << (resolutionMatch ? "✓ YES" : "✗ NO") << "\n";

    oss << "Baseline: " << std::fixed << std::setprecision(2) << baselineMM << " mm\n";
    oss << "Epipolar cy difference: " << std::setprecision(3) << cyDifference << " px\n";

    oss << "─────────────────────────────────────────────────────────────────\n";
    if (allChecksPass) {
        oss << "✓✓✓ ALL VALIDATION CHECKS PASSED ✓✓✓\n";
    } else {
        oss << "✗✗✗ VALIDATION FAILED ✗✗✗\n";
        oss << "Errors:\n" << errorMessage;
    }
    oss << "═════════════════════════════════════════════════════════════════\n";

    return oss.str();
}

} // namespace calibration
} // namespace unlook
