/**
 * @file CalibrationManager.cpp
 * @brief High-precision stereo camera calibration management implementation
 * 
 * Industrial-grade calibration system for Unlook 3D Scanner
 * Target precision: 0.005mm repeatability, <0.2 pixel RMS error
 */

#include "unlook/calibration/CalibrationManager.hpp"
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <fstream>
#include <sstream>
#include <chrono>
#include <iomanip>

namespace unlook {
namespace calibration {

// ============================================================================
// StereoCalibrationData Implementation
// ============================================================================

void StereoCalibrationData::clear() {
    // Clear all matrices
    cameraMatrixLeft = cv::Mat();
    cameraMatrixRight = cv::Mat();
    distCoeffsLeft = cv::Mat();
    distCoeffsRight = cv::Mat();
    R = cv::Mat();
    T = cv::Mat();
    E = cv::Mat();
    F = cv::Mat();
    R1 = cv::Mat();
    R2 = cv::Mat();
    P1 = cv::Mat();
    P2 = cv::Mat();
    Q = cv::Mat();
    map1Left = cv::Mat();
    map2Left = cv::Mat();
    map1Right = cv::Mat();
    map2Right = cv::Mat();
    
    // Reset metrics
    rmsError = 0.0;
    baselineMm = 0.0;
    precisionMm = 0.0;
    
    // Clear metadata
    calibrationType.clear();
    timestamp.clear();
    
    // Reset flags
    isValid = false;
    rectificationMapsComputed = false;
}

bool StereoCalibrationData::validate() const {
    // Check essential matrices are not empty
    if (cameraMatrixLeft.empty() || cameraMatrixRight.empty()) {
        return false;
    }
    if (distCoeffsLeft.empty() || distCoeffsRight.empty()) {
        return false;
    }
    if (R.empty() || T.empty()) {
        return false;
    }
    
    // Check matrix dimensions
    if (cameraMatrixLeft.size() != cv::Size(3, 3) || 
        cameraMatrixRight.size() != cv::Size(3, 3)) {
        return false;
    }
    
    // Check rotation matrix is valid (determinant should be 1)
    if (R.size() == cv::Size(3, 3)) {
        double det = cv::determinant(R);
        if (std::abs(det - 1.0) > 0.01) {
            return false;
        }
    }
    
    // Check baseline is reasonable (between 50-100mm for typical stereo system)
    double baseline = computeBaseline();
    if (baseline < 30.0 || baseline > 150.0) {
        return false;
    }
    
    // Check image size is valid
    if (imageSize.width <= 0 || imageSize.height <= 0) {
        return false;
    }
    
    return true;
}

double StereoCalibrationData::computeBaseline() const {
    if (T.empty()) {
        return 0.0;
    }
    
    // Compute baseline as the norm of the translation vector
    return cv::norm(T);
}

// ============================================================================
// CalibrationManager::Impl Implementation
// ============================================================================

class CalibrationManager::Impl {
public:
    StereoCalibrationData calibData;
    mutable std::string lastError;  // Make lastError mutable for const methods
    mutable std::mutex dataMutex;
    
    // Nominal baseline for validation (70mm for Unlook scanner)
    const double nominalBaselineMm = 70.0;
    
    Impl() {
        // Initialize with IMX296 default resolution
        calibData.imageSize = cv::Size(1456, 1088);
    }
    
    bool loadFromYAML(const std::string& filePath) {
        try {
            cv::FileStorage fs(filePath, cv::FileStorage::READ);
            if (!fs.isOpened()) {
                lastError = "Failed to open calibration file: " + filePath;
                return false;
            }
            
            // Clear existing data
            calibData.clear();
            
            // Read calibration type and metadata
            fs["calibration_type"] >> calibData.calibrationType;
            fs["timestamp"] >> calibData.timestamp;
            
            // Read quality metrics
            fs["rms_error"] >> calibData.rmsError;
            fs["baseline_mm"] >> calibData.baselineMm;
            fs["precision_mm"] >> calibData.precisionMm;
            
            // Read image size
            std::vector<int> imgSize;
            fs["image_size"] >> imgSize;
            if (imgSize.size() == 2) {
                calibData.imageSize = cv::Size(imgSize[0], imgSize[1]);
            }
            
            // Read camera matrices
            fs["camera_matrix_left"] >> calibData.cameraMatrixLeft;
            fs["camera_matrix_right"] >> calibData.cameraMatrixRight;
            
            // Read distortion coefficients
            fs["dist_coeffs_left"] >> calibData.distCoeffsLeft;
            fs["dist_coeffs_right"] >> calibData.distCoeffsRight;
            
            // Read stereo calibration data
            fs["R"] >> calibData.R;
            fs["T"] >> calibData.T;
            fs["E"] >> calibData.E;
            fs["F"] >> calibData.F;
            
            // Read rectification data
            fs["R1"] >> calibData.R1;
            fs["R2"] >> calibData.R2;
            fs["P1"] >> calibData.P1;
            fs["P2"] >> calibData.P2;
            fs["Q"] >> calibData.Q;

            // DEBUG: Verify rectification matrices were loaded
            std::cout << "[CalibrationManager] After loading from YAML:" << std::endl;
            std::cout << "  R1 empty: " << calibData.R1.empty() << ", size: " << calibData.R1.size() << std::endl;
            std::cout << "  R2 empty: " << calibData.R2.empty() << ", size: " << calibData.R2.size() << std::endl;
            std::cout << "  P1 empty: " << calibData.P1.empty() << ", size: " << calibData.P1.size() << std::endl;
            std::cout << "  P2 empty: " << calibData.P2.empty() << ", size: " << calibData.P2.size() << std::endl;
            std::cout << "  Q empty: " << calibData.Q.empty() << ", size: " << calibData.Q.size() << std::endl;

            fs.release();
            
            // Compute baseline from T vector if not explicitly stored
            if (calibData.baselineMm == 0.0 && !calibData.T.empty()) {
                calibData.baselineMm = calibData.computeBaseline();
            }
            
            // Set default values if not present
            if (calibData.calibrationType.empty()) {
                calibData.calibrationType = "OpenCV_Standard";
            }
            if (calibData.timestamp.empty()) {
                auto now = std::chrono::system_clock::now();
                auto time_t = std::chrono::system_clock::to_time_t(now);
                std::stringstream ss;
                ss << std::put_time(std::localtime(&time_t), "%Y-%m-%d %H:%M:%S");
                calibData.timestamp = ss.str();
            }
            
            // Validate the loaded calibration
            if (!calibData.validate()) {
                lastError = "Loaded calibration failed validation checks";
                return false;
            }
            
            calibData.isValid = true;
            
            std::cout << "[CalibrationManager] Successfully loaded calibration from: " << filePath << std::endl;
            std::cout << "  - Type: " << calibData.calibrationType << std::endl;
            std::cout << "  - Baseline: " << calibData.baselineMm << " mm" << std::endl;
            std::cout << "  - RMS Error: " << calibData.rmsError << " pixels" << std::endl;
            std::cout << "  - Image Size: " << calibData.imageSize.width << "x" << calibData.imageSize.height << std::endl;
            
            return true;
            
        } catch (const cv::Exception& e) {
            lastError = "OpenCV error loading calibration: " + std::string(e.what());
            calibData.isValid = false;
            return false;
        } catch (const std::exception& e) {
            lastError = "Error loading calibration: " + std::string(e.what());
            calibData.isValid = false;
            return false;
        }
    }
    
    bool saveToYAML(const std::string& filePath) const {
        if (!calibData.isValid) {
            lastError = "No valid calibration to save";
            return false;
        }
        
        try {
            cv::FileStorage fs(filePath, cv::FileStorage::WRITE);
            if (!fs.isOpened()) {
                lastError = "Failed to open file for writing: " + filePath;
                return false;
            }
            
            // Write metadata
            fs << "calibration_type" << calibData.calibrationType;
            fs << "timestamp" << calibData.timestamp;
            
            // Write quality metrics
            fs << "rms_error" << calibData.rmsError;
            fs << "baseline_mm" << calibData.baselineMm;
            fs << "precision_mm" << calibData.precisionMm;
            
            // Write image size
            fs << "image_size" << "[" << calibData.imageSize.width << calibData.imageSize.height << "]";
            
            // Write camera matrices
            fs << "camera_matrix_left" << calibData.cameraMatrixLeft;
            fs << "camera_matrix_right" << calibData.cameraMatrixRight;
            
            // Write distortion coefficients
            fs << "dist_coeffs_left" << calibData.distCoeffsLeft;
            fs << "dist_coeffs_right" << calibData.distCoeffsRight;
            
            // Write stereo calibration data
            fs << "R" << calibData.R;
            fs << "T" << calibData.T;
            fs << "E" << calibData.E;
            fs << "F" << calibData.F;
            
            // Write rectification data
            fs << "R1" << calibData.R1;
            fs << "R2" << calibData.R2;
            fs << "P1" << calibData.P1;
            fs << "P2" << calibData.P2;
            fs << "Q" << calibData.Q;
            
            fs.release();
            
            std::cout << "[CalibrationManager] Saved calibration to: " << filePath << std::endl;
            return true;
            
        } catch (const cv::Exception& e) {
            lastError = "Failed to save calibration: " + std::string(e.what());
            return false;
        }
    }
    
    bool computeRectificationMapsInternal(double alpha = 0.0) {
        if (!calibData.isValid) {
            lastError = "No valid calibration for computing rectification maps";
            return false;
        }
        
        try {
            // DEBUG: Check if rectification matrices need to be computed
            std::cout << "[CalibrationManager] computeRectificationMapsInternal called with alpha=" << alpha << std::endl;
            std::cout << "  R1 empty: " << calibData.R1.empty() << ", R2 empty: " << calibData.R2.empty() << std::endl;
            std::cout << "  P1 empty: " << calibData.P1.empty() << ", P2 empty: " << calibData.P2.empty() << std::endl;

            // CRITICAL: Verify P1/P2 principal points (cy MUST be identical for horizontal epipolar lines)
            if (!calibData.P1.empty() && !calibData.P2.empty()) {
                double cx_left = calibData.P1.at<double>(0, 2);
                double cy_left = calibData.P1.at<double>(1, 2);
                double fx_left = calibData.P1.at<double>(0, 0);
                double fy_left = calibData.P1.at<double>(1, 1);

                double cx_right = calibData.P2.at<double>(0, 2);
                double cy_right = calibData.P2.at<double>(1, 2);
                double fx_right = calibData.P2.at<double>(0, 0);
                double fy_right = calibData.P2.at<double>(1, 1);

                std::cout << "  ===== RECTIFIED PROJECTION MATRICES ANALYSIS =====" << std::endl;
                std::cout << "  P1 (LEFT):  fx=" << fx_left << ", fy=" << fy_left
                         << ", cx=" << cx_left << ", cy=" << cy_left << std::endl;
                std::cout << "  P2 (RIGHT): fx=" << fx_right << ", fy=" << fy_right
                         << ", cx=" << cx_right << ", cy=" << cy_right << std::endl;
                std::cout << "  cy difference (RIGHT-LEFT): " << (cy_right - cy_left) << " pixels" << std::endl;

                if (std::abs(cy_right - cy_left) > 0.1) {
                    std::cerr << "  ⚠️  WARNING: cy offset detected! Epipolar lines will NOT be aligned!" << std::endl;
                    std::cerr << "  ⚠️  This causes SGBM to search on wrong scanlines → incorrect disparity!" << std::endl;
                } else {
                    std::cout << "  ✓ cy values are aligned (offset < 0.1px)" << std::endl;
                }
                std::cout << "  ==================================================" << std::endl;
            }

            // If rectification matrices are not available, compute them
            if (calibData.R1.empty() || calibData.R2.empty() ||
                calibData.P1.empty() || calibData.P2.empty()) {

                std::cout << "[CalibrationManager] WARNING: Rectification matrices missing! RECALCULATING with stereoRectify!" << std::endl;
                std::cout << "  THIS IS WRONG - Should use pre-computed BoofCV matrices!" << std::endl;

                cv::Rect validRoi1, validRoi2;

                cv::stereoRectify(
                    calibData.cameraMatrixLeft, calibData.distCoeffsLeft,
                    calibData.cameraMatrixRight, calibData.distCoeffsRight,
                    calibData.imageSize,
                    calibData.R, calibData.T,
                    calibData.R1, calibData.R2,
                    calibData.P1, calibData.P2,
                    calibData.Q,
                    cv::CALIB_ZERO_DISPARITY,
                    alpha,
                    calibData.imageSize,
                    &validRoi1, &validRoi2
                );

                std::cout << "[CalibrationManager] RECALCULATION COMPLETE - epipolar lines will NOT match BoofCV!" << std::endl;
            } else {
                std::cout << "[CalibrationManager] Using pre-computed BoofCV rectification matrices (CORRECT!)" << std::endl;
            }

            // SWAP FIX: Calibration file has cameras swapped (Tx negative)
            // LEFT camera (Camera 1) uses R2/P2 from calibration file
            // RIGHT camera (Camera 0) uses R1/P1 from calibration file
            std::cout << "[CalibrationManager] SWAPPING R1↔R2 and P1↔P2 to fix camera mapping" << std::endl;

            // Compute rectification maps for left camera (uses R2/P2 from file)
            cv::initUndistortRectifyMap(
                calibData.cameraMatrixLeft, calibData.distCoeffsLeft,
                calibData.R2, calibData.P2,  // SWAPPED: R1→R2, P1→P2
                calibData.imageSize, CV_32FC1,
                calibData.map1Left, calibData.map2Left
            );

            // Compute rectification maps for right camera (uses R1/P1 from file)
            cv::initUndistortRectifyMap(
                calibData.cameraMatrixRight, calibData.distCoeffsRight,
                calibData.R1, calibData.P1,  // SWAPPED: R2→R1, P2→P1
                calibData.imageSize, CV_32FC1,
                calibData.map1Right, calibData.map2Right
            );
            
            calibData.rectificationMapsComputed = true;
            
            std::cout << "[CalibrationManager] Rectification maps computed successfully (alpha=" << alpha << ")" << std::endl;
            return true;
            
        } catch (const cv::Exception& e) {
            lastError = "Failed to compute rectification maps: " + std::string(e.what());
            calibData.rectificationMapsComputed = false;
            return false;
        }
    }
    
    std::string generateValidationReport() const {
        std::stringstream report;
        
        report << "===== Stereo Calibration Validation Report =====\n\n";
        
        // Basic information
        report << "Calibration Type: " << calibData.calibrationType << "\n";
        report << "Timestamp: " << calibData.timestamp << "\n";
        report << "Image Size: " << calibData.imageSize.width << "x" << calibData.imageSize.height << "\n\n";
        
        // Quality metrics
        report << "=== Quality Metrics ===\n";
        report << "RMS Reprojection Error: " << std::fixed << std::setprecision(4) 
               << calibData.rmsError << " pixels\n";
        
        // Industrial precision check (<0.2 pixel target)
        bool rmsPass = calibData.rmsError < 0.2;
        report << "  Status: " << (rmsPass ? "PASS" : "WARNING") 
               << " (target < 0.2 pixels)\n\n";
        
        // Baseline validation
        report << "Baseline: " << std::fixed << std::setprecision(3) 
               << calibData.baselineMm << " mm\n";
        
        double baselineDeviation = std::abs(calibData.baselineMm - nominalBaselineMm);
        bool baselinePass = baselineDeviation < 0.5;
        report << "  Deviation from nominal (70mm): " << baselineDeviation << " mm\n";
        report << "  Status: " << (baselinePass ? "PASS" : "WARNING") 
               << " (tolerance < 0.5mm)\n\n";
        
        // Expected precision
        report << "Expected Precision: " << std::fixed << std::setprecision(4) 
               << calibData.precisionMm << " mm\n";
        
        bool precisionPass = calibData.precisionMm < 0.005;
        report << "  Status: " << (precisionPass ? "PASS" : "WARNING") 
               << " (target < 0.005mm)\n\n";
        
        // Camera parameters validation
        report << "=== Camera Parameters ===\n";
        
        if (!calibData.cameraMatrixLeft.empty()) {
            double fx_left = calibData.cameraMatrixLeft.at<double>(0, 0);
            double fy_left = calibData.cameraMatrixLeft.at<double>(1, 1);
            double cx_left = calibData.cameraMatrixLeft.at<double>(0, 2);
            double cy_left = calibData.cameraMatrixLeft.at<double>(1, 2);
            
            report << "Left Camera:\n";
            report << "  Focal Length: fx=" << std::fixed << std::setprecision(2) 
                   << fx_left << ", fy=" << fy_left << " pixels\n";
            report << "  Principal Point: cx=" << cx_left << ", cy=" << cy_left << " pixels\n";
            
            // Check focal length symmetry
            double focalAsymmetry = std::abs(fx_left - fy_left) / fx_left * 100;
            report << "  Focal Length Asymmetry: " << std::fixed << std::setprecision(2) 
                   << focalAsymmetry << "%\n";
        }
        
        if (!calibData.cameraMatrixRight.empty()) {
            double fx_right = calibData.cameraMatrixRight.at<double>(0, 0);
            double fy_right = calibData.cameraMatrixRight.at<double>(1, 1);
            double cx_right = calibData.cameraMatrixRight.at<double>(0, 2);
            double cy_right = calibData.cameraMatrixRight.at<double>(1, 2);
            
            report << "Right Camera:\n";
            report << "  Focal Length: fx=" << std::fixed << std::setprecision(2) 
                   << fx_right << ", fy=" << fy_right << " pixels\n";
            report << "  Principal Point: cx=" << cx_right << ", cy=" << cy_right << " pixels\n";
            
            // Check focal length symmetry
            double focalAsymmetry = std::abs(fx_right - fy_right) / fx_right * 100;
            report << "  Focal Length Asymmetry: " << std::fixed << std::setprecision(2) 
                   << focalAsymmetry << "%\n";
        }
        
        report << "\n";
        
        // Stereo geometry validation
        report << "=== Stereo Geometry ===\n";
        
        if (!calibData.R.empty()) {
            double det = cv::determinant(calibData.R);
            report << "Rotation Matrix Determinant: " << std::fixed << std::setprecision(6) 
                   << det << " (should be 1.0)\n";
            
            bool rotationValid = std::abs(det - 1.0) < 0.01;
            report << "  Status: " << (rotationValid ? "PASS" : "FAIL") << "\n";
        }
        
        if (!calibData.E.empty() && !calibData.F.empty()) {
            // Check if essential and fundamental matrices are non-zero
            double normE = cv::norm(calibData.E);
            double normF = cv::norm(calibData.F);
            
            report << "Essential Matrix Norm: " << std::scientific << normE << "\n";
            report << "Fundamental Matrix Norm: " << std::scientific << normF << "\n";
            
            bool matricesValid = normE > 1e-10 && normF > 1e-10;
            report << "  Status: " << (matricesValid ? "PASS" : "FAIL") << "\n";
        }
        
        report << "\n";
        
        // Rectification status
        report << "=== Rectification Status ===\n";
        report << "Rectification Maps Computed: " 
               << (calibData.rectificationMapsComputed ? "Yes" : "No") << "\n";
        
        if (!calibData.P1.empty() && !calibData.P2.empty()) {
            // Extract baseline from projection matrices
            double baseline_px = std::abs(calibData.P2.at<double>(0, 3));
            double fx = calibData.P1.at<double>(0, 0);
            double baseline_computed = baseline_px / fx;
            
            report << "Baseline from Projection Matrices: " << std::fixed << std::setprecision(3) 
                   << baseline_computed << " mm\n";
        }
        
        report << "\n";
        
        // Overall assessment
        report << "=== Overall Assessment ===\n";
        
        bool overallPass = rmsPass && baselinePass && calibData.isValid;
        
        if (overallPass) {
            report << "Status: PASS - Calibration meets industrial precision requirements\n";
        } else {
            report << "Status: NEEDS IMPROVEMENT\n";
            report << "Recommendations:\n";
            
            if (!rmsPass) {
                report << "  - RMS error exceeds target. Consider:\n";
                report << "    * Capturing more calibration images (50+ recommended)\n";
                report << "    * Ensuring better pose distribution\n";
                report << "    * Checking calibration target quality\n";
            }
            
            if (!baselinePass) {
                report << "  - Baseline deviation exceeds tolerance. Consider:\n";
                report << "    * Verifying physical camera mounting\n";
                report << "    * Re-measuring actual baseline\n";
                report << "    * Checking for mechanical shifts\n";
            }
            
            if (!precisionPass) {
                report << "  - Precision exceeds target. Consider:\n";
                report << "    * Improving calibration quality\n";
                report << "    * Using higher resolution cameras\n";
                report << "    * Optimizing stereo matching parameters\n";
            }
        }
        
        report << "\n===== End of Report =====\n";
        
        return report.str();
    }
};

// ============================================================================
// CalibrationManager Implementation
// ============================================================================

CalibrationManager::CalibrationManager() 
    : pImpl(std::make_unique<Impl>()) 
{
    std::cout << "[CalibrationManager] Initialized for Unlook 3D Scanner (70mm baseline stereo system)" << std::endl;
    std::cout << "[CalibrationManager] Target precision: 0.005mm, RMS < 0.2 pixels" << std::endl;
}

CalibrationManager::~CalibrationManager() = default;

bool CalibrationManager::loadCalibration(const std::string& filePath) {
    std::lock_guard<std::mutex> lock(pImpl->dataMutex);
    
    bool success = pImpl->loadFromYAML(filePath);
    
    if (success) {
        // Automatically compute rectification maps after loading
        pImpl->computeRectificationMapsInternal(0.0);
    }
    
    return success;
}

bool CalibrationManager::saveCalibration(const std::string& filePath) const {
    std::lock_guard<std::mutex> lock(pImpl->dataMutex);
    return pImpl->saveToYAML(filePath);
}

const StereoCalibrationData& CalibrationManager::getCalibrationData() const {
    std::lock_guard<std::mutex> lock(pImpl->dataMutex);
    return pImpl->calibData;
}

bool CalibrationManager::isCalibrationValid() const {
    std::lock_guard<std::mutex> lock(pImpl->dataMutex);
    return pImpl->calibData.isValid;
}

bool CalibrationManager::computeRectificationMaps(double alpha) {
    std::lock_guard<std::mutex> lock(pImpl->dataMutex);
    return pImpl->computeRectificationMapsInternal(alpha);
}

bool CalibrationManager::rectifyImages(const cv::Mat& leftImage, const cv::Mat& rightImage,
                                       cv::Mat& leftRectified, cv::Mat& rightRectified) {
    std::lock_guard<std::mutex> lock(pImpl->dataMutex);
    
    if (!pImpl->calibData.isValid) {
        pImpl->lastError = "No valid calibration for rectification";
        return false;
    }
    
    if (!pImpl->calibData.rectificationMapsComputed) {
        // Try to compute rectification maps
        if (!pImpl->computeRectificationMapsInternal(0.0)) {
            return false;
        }
    }
    
    try {
        // Apply rectification using pre-computed maps
        cv::remap(leftImage, leftRectified,
                  pImpl->calibData.map1Left, pImpl->calibData.map2Left,
                  cv::INTER_LINEAR);
        
        cv::remap(rightImage, rightRectified,
                  pImpl->calibData.map1Right, pImpl->calibData.map2Right,
                  cv::INTER_LINEAR);
        
        return true;
        
    } catch (const cv::Exception& e) {
        pImpl->lastError = "Rectification failed: " + std::string(e.what());
        return false;
    }
}

double CalibrationManager::getRmsError() const {
    std::lock_guard<std::mutex> lock(pImpl->dataMutex);
    return pImpl->calibData.rmsError;
}

double CalibrationManager::getBaselineMm() const {
    std::lock_guard<std::mutex> lock(pImpl->dataMutex);
    return pImpl->calibData.baselineMm;
}

double CalibrationManager::getPrecisionMm() const {
    std::lock_guard<std::mutex> lock(pImpl->dataMutex);
    return pImpl->calibData.precisionMm;
}

bool CalibrationManager::validateCalibrationQuality(double maxRmsError, double maxBaselineDeviation) const {
    std::lock_guard<std::mutex> lock(pImpl->dataMutex);
    
    if (!pImpl->calibData.isValid) {
        return false;
    }
    
    // Check RMS error
    if (pImpl->calibData.rmsError > maxRmsError) {
        std::cout << "[CalibrationManager] RMS error (" << pImpl->calibData.rmsError 
                  << ") exceeds limit (" << maxRmsError << ")" << std::endl;
        return false;
    }
    
    // Check baseline deviation
    double baselineDeviation = std::abs(pImpl->calibData.baselineMm - pImpl->nominalBaselineMm);
    if (baselineDeviation > maxBaselineDeviation) {
        std::cout << "[CalibrationManager] Baseline deviation (" << baselineDeviation 
                  << "mm) exceeds limit (" << maxBaselineDeviation << "mm)" << std::endl;
        return false;
    }
    
    // Check essential and fundamental matrices
    if (pImpl->calibData.E.empty() || pImpl->calibData.F.empty()) {
        std::cout << "[CalibrationManager] Essential or Fundamental matrix is empty" << std::endl;
        return false;
    }
    
    double normE = cv::norm(pImpl->calibData.E);
    double normF = cv::norm(pImpl->calibData.F);
    
    if (normE < 1e-10 || normF < 1e-10) {
        std::cout << "[CalibrationManager] Essential or Fundamental matrix is near-zero" << std::endl;
        return false;
    }
    
    return true;
}

std::string CalibrationManager::getValidationReport() const {
    std::lock_guard<std::mutex> lock(pImpl->dataMutex);
    return pImpl->generateValidationReport();
}

cv::Point3f CalibrationManager::convertTo3D(const cv::Point2f& point, float disparity) const {
    std::lock_guard<std::mutex> lock(pImpl->dataMutex);
    
    if (!pImpl->calibData.isValid || pImpl->calibData.Q.empty()) {
        return cv::Point3f(0, 0, 0);
    }
    
    // Use Q matrix to convert from disparity to 3D
    // Q matrix formula: [X Y Z W]' = Q * [x y disparity 1]'
    cv::Mat vec = (cv::Mat_<double>(4, 1) << point.x, point.y, disparity, 1.0);
    cv::Mat result = pImpl->calibData.Q * vec;
    
    double W = result.at<double>(3, 0);
    if (std::abs(W) < 1e-10) {
        return cv::Point3f(0, 0, 0);
    }
    
    return cv::Point3f(
        static_cast<float>(result.at<double>(0, 0) / W),
        static_cast<float>(result.at<double>(1, 0) / W),
        static_cast<float>(result.at<double>(2, 0) / W)
    );
}

void CalibrationManager::computeFieldOfView(double& fovX, double& fovY) const {
    std::lock_guard<std::mutex> lock(pImpl->dataMutex);
    
    if (!pImpl->calibData.isValid || pImpl->calibData.cameraMatrixLeft.empty()) {
        fovX = 0.0;
        fovY = 0.0;
        return;
    }
    
    double fx = pImpl->calibData.cameraMatrixLeft.at<double>(0, 0);
    double fy = pImpl->calibData.cameraMatrixLeft.at<double>(1, 1);
    double cx = pImpl->calibData.cameraMatrixLeft.at<double>(0, 2);
    double cy = pImpl->calibData.cameraMatrixLeft.at<double>(1, 2);
    
    // Compute field of view in radians, then convert to degrees
    fovX = 2.0 * std::atan(pImpl->calibData.imageSize.width / (2.0 * fx)) * 180.0 / CV_PI;
    fovY = 2.0 * std::atan(pImpl->calibData.imageSize.height / (2.0 * fy)) * 180.0 / CV_PI;
}

void CalibrationManager::computeEpipolarLines(const std::vector<cv::Point2f>& points,
                                              std::vector<cv::Vec3f>& epilines,
                                              bool useLeft) const {
    std::lock_guard<std::mutex> lock(pImpl->dataMutex);
    
    if (!pImpl->calibData.isValid || pImpl->calibData.F.empty() || points.empty()) {
        epilines.clear();
        return;
    }
    
    // Compute epipolar lines
    // If useLeft is true, compute lines in right image for left image points
    // If useLeft is false, compute lines in left image for right image points
    int whichImage = useLeft ? 1 : 2;
    cv::computeCorrespondEpilines(points, whichImage, pImpl->calibData.F, epilines);
}

} // namespace calibration
} // namespace unlook