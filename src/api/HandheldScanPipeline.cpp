/**
 * @file HandheldScanPipeline.cpp
 * @brief PRODUCTION handheld scanning pipeline with SGM-Census stereo matching
 *
 * Complete rewrite using production-grade SGMCensus algorithm.
 * Removes all fantasy dependencies (RectificationEngine, DisparityComputer, etc.)
 * Uses OpenCV standard functions + SGMCensus for industrial quality.
 *
 * @author Alessandro (Unlook Project)
 * @date 2025-11-12
 * @version 2.0 - Production Implementation
 */

#include <unlook/api/HandheldScanPipeline.hpp>
#include <unlook/api/camera_system.h>
#include <unlook/core/Logger.hpp>
#include <unlook/core/exception.h>
#include <unlook/stereo/SGMCensus.hpp>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/ximgproc.hpp>
#include <opencv2/imgcodecs.hpp>

#include <algorithm>
#include <numeric>
#include <thread>
#include <fstream>
#include <filesystem>

namespace fs = std::filesystem;

namespace unlook {
namespace api {

/**
 * @brief Private implementation - PRODUCTION SGMCensus
 */
class HandheldScanPipeline::Impl {
public:
    // ========== CORE COMPONENTS ==========
    std::shared_ptr<camera::CameraSystem> cameraSystem_;
    core::Logger& logger_;

    // SGMCensus stereo matcher
    std::unique_ptr<stereo::SGMCensus> sgmCensus_;
    stereo::SGMCensus::Config censusConfig_;

    // ========== CALIBRATION DATA ==========
    cv::Mat cameraMatrixLeft_, cameraMatrixRight_;
    cv::Mat distCoeffsLeft_, distCoeffsRight_;
    cv::Mat R_, T_;  // Rotation and translation between cameras
    cv::Mat R1_, R2_, P1_, P2_;  // Rectification transforms
    cv::Mat map1Left_, map2Left_;    // Undistort + rectify maps for left camera
    cv::Mat map1Right_, map2Right_;  // Undistort + rectify maps for right camera
    cv::Size imageSize_;
    float baseline_mm_ = 70.017f;
    cv::Mat Q_;  // Disparity-to-depth matrix

    // ========== CENTER CROP FOR INVESTOR DEMO ==========
    // Based on MATLAB FINAL calibration analysis:
    // - Full image (1280x720): 65.45px epipolar error (POOR)
    // - CENTER crop (680x420): 1.99px epipolar error (EXCELLENT)
    // Crop centered on rectified principal point (656.11, 326.00) for optimal alignment
    bool useCenterCrop_ = true;  // Enable for investor demo
    int cropLeft_ = 316;   // Centered on cx_rect=656.11 (was 300)
    int cropRight_ = 284;  // Asymmetric to maintain center (was 300)
    int cropTop_ = 116;    // Centered on cy_rect=326.00 (was 150)
    int cropBottom_ = 184; // Asymmetric to maintain center (was 150)
    cv::Size croppedSize_ = cv::Size(680, 420);  // 31% of original area

    // ========== RECTANGULAR BORDER FILTER ==========
    // CRITICAL: Image borders are RECTANGULAR (680x420), not circular!
    // Radial filter FAILS because circle doesn't cover rectangular edges:
    //   - Point (340, 0) is 210px from center → inside circle → NOT FILTERED ❌
    //   - Point (0, 210) is 340px from center → inside circle → NOT FILTERED ❌
    // Solution: Filter RECTANGULAR margin from all 4 edges (top/bottom/left/right)
    //
    // Border margin calculation:
    //   - Image size: 680x420
    //   - Margin 30px → keeps 620x360 (81% area)
    //   - Margin 40px → keeps 600x340 (74% area)
    //   - Margin 50px → keeps 580x320 (68% area)
    bool enableBorderFilter_ = false;             // DISABLED: Enable rectangular border filtering
    int borderMarginPixels_ = 40;                 // Filter 40px from all edges (adjust 30-50)

    // ========== STATISTICAL OUTLIER REMOVAL ==========
    // Removes diffuse noise cloud (outlier points scattered in empty space)
    // Method: For each point, compute distance to K nearest neighbors
    // If mean distance > threshold * stddev → outlier
    //
    // OPTIMIZED for REAL-TIME (RPi5):
    //   - k=12: Reduced from 30 (k²: 900→144 iterations, 6x faster)
    //   - Multi-threading: Parallel row processing on 4 Cortex-A76 cores
    //   - Expected: ~500ms → ~50ms (10x speedup with k+threading)
    //
    // Parameters tuning:
    //   - k=10, threshold=2.0 → fast, aggressive (removes 5-10% outliers)
    //   - k=12, threshold=1.5 → balanced (removes 8-12% outliers) ← CURRENT
    //   - k=15, threshold=1.2 → slower, very aggressive (removes 12-15% outliers)
    bool enableStatisticalFilter_ = false;        // DISABLED: Enable statistical outlier removal
    int statisticalFilterK_ = 12;                 // Neighborhood size (OPTIMIZED: 12 instead of 30)
    float statisticalFilterThreshold_ = 1.5f;     // Std dev multiplier (lower = more aggressive)

    // ========== DEBUG & STATISTICS ==========
    bool saveDebugImages_ = false;
    std::string debugDir_;
    ProgressCallback currentProgressCallback_ = nullptr;

    Impl(std::shared_ptr<camera::CameraSystem> cameraSystem)
        : cameraSystem_(cameraSystem), logger_(core::Logger::getInstance())
    {
        // Initialize SGMCensus with 9x9 Census (like original epiception/SGM-Census repo)
        censusConfig_.censusWindowSize = 9;   // 9x9 = 80 bits (EXACT match to epiception repo)
        censusConfig_.numDisparities = 384;
        censusConfig_.P1 = 8;     // Standard SGM penalty (small disparity change)
        censusConfig_.P2 = 24;    // REDUCED from 32 → allows more disparity jumps → more valid pixels
        censusConfig_.use8Paths = true;
        censusConfig_.verticalSearchRange = 2;  // ENABLED: ±2px vertical search for epipolar errors
        censusConfig_.verbose = false;  // Disable for production

        sgmCensus_ = std::make_unique<stereo::SGMCensus>(censusConfig_);

        logger_.info("[HandheldScanPipeline] Initialized with SGMCensus (census=9x9, P1=8, P2=24, vertical=±2px, CLAHE=4.0)");

        // Log CENTER crop status for investor demo
        if (useCenterCrop_) {
            logger_.info("[HandheldScanPipeline] CENTER crop ENABLED for investor demo");
            logger_.info("[HandheldScanPipeline]   Crop size: " + std::to_string(croppedSize_.width) + "x" + std::to_string(croppedSize_.height));
            logger_.info("[HandheldScanPipeline]   Crop centered on principal point (656, 326)");
            logger_.info("[HandheldScanPipeline]   Margins: L=" + std::to_string(cropLeft_) + ", R=" + std::to_string(cropRight_) +
                        ", T=" + std::to_string(cropTop_) + ", B=" + std::to_string(cropBottom_));
            logger_.info("[HandheldScanPipeline]   Expected epipolar error: ~2px (vs 65px full image)");

            // Log filter configuration
            if (enableBorderFilter_) {
                int validWidth = croppedSize_.width - 2 * borderMarginPixels_;
                int validHeight = croppedSize_.height - 2 * borderMarginPixels_;
                float validArea = (100.0f * validWidth * validHeight) / (croppedSize_.width * croppedSize_.height);

                logger_.info("[HandheldScanPipeline] RECTANGULAR BORDER FILTER ENABLED");
                logger_.info("[HandheldScanPipeline]   Border margin: " + std::to_string(borderMarginPixels_) + "px from all edges");
                logger_.info("[HandheldScanPipeline]   Valid region: " + std::to_string(validWidth) + "x" + std::to_string(validHeight) +
                            " (" + std::to_string(validArea) + "% of crop area)");
            }
            if (enableStatisticalFilter_) {
                logger_.info("[HandheldScanPipeline] STATISTICAL OUTLIER REMOVAL ENABLED");
                logger_.info("[HandheldScanPipeline]   Neighborhood: " + std::to_string(statisticalFilterK_) + "px");
                logger_.info("[HandheldScanPipeline]   Threshold: " + std::to_string(statisticalFilterThreshold_) + " * std_dev");
            }
        } else {
            logger_.info("[HandheldScanPipeline] CENTER crop DISABLED (using full image)");
        }
    }

    ~Impl() {
        logger_.info("[HandheldScanPipeline] Shutting down");
    }

    bool loadCalibration(const std::string& calibPath) {
        try {
            // Check if default.yaml symlink exists
            const std::string defaultCalib = "/unlook_calib/default.yaml";
            std::string actualPath = calibPath.empty() ? defaultCalib : calibPath;

            if (fs::exists(defaultCalib)) {
                logger_.info("[HandheldScanPipeline] Found default.yaml symlink: " + defaultCalib);
                logger_.info("[HandheldScanPipeline] Auto-loading latest calibration: " + defaultCalib);
                actualPath = defaultCalib;
            }

            logger_.info("[HandheldScanPipeline] Loading calibration from: " + actualPath);

            cv::FileStorage fs(actualPath, cv::FileStorage::READ);
            if (!fs.isOpened()) {
                logger_.error("[HandheldScanPipeline] Failed to open calibration file: " + actualPath);
                return false;
            }

            // Load camera matrices
            fs["camera_matrix_left"] >> cameraMatrixLeft_;
            fs["camera_matrix_right"] >> cameraMatrixRight_;
            fs["distortion_coeffs_left"] >> distCoeffsLeft_;   // FIX: Correct YAML key name
            fs["distortion_coeffs_right"] >> distCoeffsRight_; // FIX: Correct YAML key name

            // Load stereo transform
            fs["rotation_matrix"] >> R_;
            fs["translation_vector"] >> T_;

            // Calculate baseline
            baseline_mm_ = cv::norm(T_);
            logger_.info("[HandheldScanPipeline] Baseline (from translation): " +
                        std::to_string(baseline_mm_) + " mm");

            // Get image size
            int width = fs["image_width"];
            int height = fs["image_height"];
            imageSize_ = cv::Size(width, height);

            // Try to load precomputed rectification
            cv::Mat rectLeft, rectRight, projLeft, projRight, Q;
            fs["rectification_transform_left"] >> rectLeft;
            fs["rectification_transform_right"] >> rectRight;
            fs["projection_matrix_left"] >> projLeft;
            fs["projection_matrix_right"] >> projRight;
            fs["disparity_to_depth_matrix"] >> Q;

            if (!rectLeft.empty() && !rectRight.empty() && !projLeft.empty() && !projRight.empty() && !Q.empty()) {
                // Use precomputed rectification from calibration file
                logger_.info("[HandheldScanPipeline] Using precomputed rectification from calibration");
                R1_ = rectLeft;
                R2_ = rectRight;
                P1_ = projLeft;
                P2_ = projRight;
                Q_ = Q;
            } else {
                // Compute rectification transforms if not precomputed
                logger_.info("[HandheldScanPipeline] Computing stereo rectification...");
                cv::stereoRectify(
                    cameraMatrixLeft_, distCoeffsLeft_,
                    cameraMatrixRight_, distCoeffsRight_,
                    imageSize_,
                    R_, T_,
                    R1_, R2_, P1_, P2_, Q_,
                    cv::CALIB_ZERO_DISPARITY,
                    1,  // alpha=1 (matching OpenCV sample)
                    imageSize_
                );
            }

            fs.release();

            // Precompute rectification maps for fast remapping
            logger_.info("[HandheldScanPipeline] Computing rectification maps...");
            // FIX: Use CV_16SC2 format (more efficient, matching calibration)
            cv::initUndistortRectifyMap(
                cameraMatrixLeft_, distCoeffsLeft_, R1_, P1_,
                imageSize_, CV_16SC2, map1Left_, map2Left_
            );
            cv::initUndistortRectifyMap(
                cameraMatrixRight_, distCoeffsRight_, R2_, P2_,
                imageSize_, CV_16SC2, map1Right_, map2Right_
            );

            logger_.info("[HandheldScanPipeline] Calibration loaded successfully");
            logger_.info("[HandheldScanPipeline]   Image size: " +
                        std::to_string(imageSize_.width) + "x" +
                        std::to_string(imageSize_.height));
            logger_.info("[HandheldScanPipeline]   Baseline: " +
                        std::to_string(baseline_mm_) + " mm");

            return true;

        } catch (const cv::Exception& e) {
            logger_.error("[HandheldScanPipeline] OpenCV exception loading calibration: " +
                         std::string(e.what()));
            return false;
        } catch (const std::exception& e) {
            logger_.error("[HandheldScanPipeline] Exception loading calibration: " +
                         std::string(e.what()));
            return false;
        }
    }

    void rectifyStereoPair(const cv::Mat& leftRaw, const cv::Mat& rightRaw,
                          cv::Mat& leftRect, cv::Mat& rightRect) {
        // Apply precomputed rectification maps (very fast)
        cv::remap(leftRaw, leftRect, map1Left_, map2Left_, cv::INTER_LINEAR);
        cv::remap(rightRaw, rightRect, map1Right_, map2Right_, cv::INTER_LINEAR);
    }

    std::vector<cv::Mat> processFrames(
        const std::vector<HandheldScanPipeline::StereoFrame>& frames,
        const stereo::StereoMatchingParams& params,
        ProgressCallback progressCallback)
    {
        std::vector<cv::Mat> disparityMaps;
        disparityMaps.reserve(frames.size());

        logger_.info("[HandheldScanPipeline] Processing " + std::to_string(frames.size()) + " stereo frames with SGMCensus");

        for (size_t i = 0; i < frames.size(); i++) {
            if (progressCallback) {
                float progress = static_cast<float>(i) / frames.size();
                std::string msg = "Processing frame " + std::to_string(i + 1) + "/" + std::to_string(frames.size());
                progressCallback(progress, msg);
            }

            // Rectify stereo pair
            cv::Mat leftRect, rightRect;
            rectifyStereoPair(frames[i].leftImage, frames[i].rightImage, leftRect, rightRect);

            // Save debug images (FULL rectified) if enabled
            if (saveDebugImages_ && !debugDir_.empty()) {
                std::string frameNum = std::to_string(i);
                cv::imwrite(debugDir_ + "/01_rectified_full_frame" + frameNum + "_left.png", leftRect);
                cv::imwrite(debugDir_ + "/01_rectified_full_frame" + frameNum + "_right.png", rightRect);
            }

            // CRITICAL OPTIMIZATION: PHYSICAL CROP (69% memory/processing saving!)
            // Previous approach: MASK with black borders → wasted 69% memory + Census on useless pixels
            // New approach: PHYSICAL CROP → only process valid 680x420 region
            //
            // Tested with MATLAB calibration:
            // - Full image: 65.45px epipolar error (POOR - insufficient edge coverage)
            // - CENTER crop: 1.99px epipolar error (EXCELLENT)
            //
            // Q matrix adjustment required:
            // - Original Q has principal point at (737.69, 364.22) in 1280x720 coords
            // - After crop, new coordinate system is 680x420 with origin at ROI top-left
            // - Principal point in crop coords: (421.69, 248.22)
            // - Crop center: (340, 210)
            // - Offset STILL 81.69px X, 38.22px Y → Q adjustment needed in generatePointCloud()
            if (useCenterCrop_) {
                cv::Rect validRoi(cropLeft_, cropTop_,
                                 croppedSize_.width, croppedSize_.height);

                // PHYSICAL CROP: Extract only the valid ROI region
                // CRITICAL: Use .clone() to create independent copy (not just a view/reference)
                leftRect = leftRect(validRoi).clone();
                rightRect = rightRect(validRoi).clone();

                // Log first frame for verification
                if (i == 0) {
                    logger_.info("[HandheldScanPipeline] PHYSICAL CROP applied (69% memory/processing reduction!)");
                    logger_.info("[HandheldScanPipeline]   Cropped to: " +
                                std::to_string(croppedSize_.width) + "x" + std::to_string(croppedSize_.height));
                    logger_.info("[HandheldScanPipeline]   From ROI: [" +
                                std::to_string(cropLeft_) + ":" + std::to_string(cropLeft_ + croppedSize_.width) +
                                ", " +
                                std::to_string(cropTop_) + ":" + std::to_string(cropTop_ + croppedSize_.height) + "]");
                    logger_.info("[HandheldScanPipeline]   Original: " +
                                std::to_string(imageSize_.width) + "x" + std::to_string(imageSize_.height) +
                                " → Cropped: " +
                                std::to_string(leftRect.cols) + "x" + std::to_string(leftRect.rows));
                    logger_.info("[HandheldScanPipeline]   New coordinate system: (0,0) at ROI top-left");
                    logger_.info("[HandheldScanPipeline]   Q matrix will be adjusted in generatePointCloud() for crop coords");
                }
            }

            // Save debug images (CROPPED) if enabled
            if (saveDebugImages_ && !debugDir_.empty() && useCenterCrop_) {
                std::string frameNum = std::to_string(i);
                cv::imwrite(debugDir_ + "/02_cropped_frame" + frameNum + "_left.png", leftRect);
                cv::imwrite(debugDir_ + "/02_cropped_frame" + frameNum + "_right.png", rightRect);
            }

            // Convert to grayscale if needed
            cv::Mat leftGray, rightGray;
            if (leftRect.channels() == 3) {
                cv::cvtColor(leftRect, leftGray, cv::COLOR_BGR2GRAY);
                cv::cvtColor(rightRect, rightGray, cv::COLOR_BGR2GRAY);
            } else {
                leftGray = leftRect;
                rightGray = rightRect;
            }

            // CRITICAL: Apply CLAHE to enhance VCSEL dot patterns
            // VCSEL structured light patterns can have low contrast in ambient light
            // CLAHE (Contrast Limited Adaptive Histogram Equalization) improves local contrast
            // Clip limit 2-4 recommended for VCSEL enhancement without over-amplifying noise
            // INCREASED to 4.0 for maximum texture enhancement → more valid disparities
            cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(4.0, cv::Size(8, 8));
            cv::Mat leftEnhanced, rightEnhanced;
            clahe->apply(leftGray, leftEnhanced);
            clahe->apply(rightGray, rightEnhanced);

            // Use enhanced images for stereo matching
            leftGray = leftEnhanced;
            rightGray = rightEnhanced;

            // Compute disparity with SGMCensus
            auto result = sgmCensus_->compute(leftGray, rightGray);

            if (!result.success) {
                logger_.error("[HandheldScanPipeline] Frame " + std::to_string(i) + " failed: " + result.errorMessage);
                continue;
            }

            logger_.info("[HandheldScanPipeline] Frame " + std::to_string(i) +
                        " - Valid: " + std::to_string(result.validPercent) +
                        "%, Time: " + std::to_string(result.totalTimeMs) + " ms");

            disparityMaps.push_back(result.disparity.clone());

            // Save debug disparity visualization
            if (saveDebugImages_ && !debugDir_.empty()) {
                cv::Mat dispVis;
                cv::normalize(result.disparity, dispVis, 0, 255, cv::NORM_MINMAX, CV_8U);
                std::string frameNum = std::to_string(i);
                cv::imwrite(debugDir_ + "/02_disparity_frame" + frameNum + ".png", dispVis);
            }
        }

        logger_.info("[HandheldScanPipeline] Processed " + std::to_string(disparityMaps.size()) +
                    " disparity maps successfully");

        return disparityMaps;
    }

    cv::Mat fuseDisparityMaps(const std::vector<cv::Mat>& disparityMaps, float outlierSigma) {
        if (disparityMaps.empty()) {
            logger_.error("[HandheldScanPipeline] No disparity maps to fuse");
            return cv::Mat();
        }

        if (disparityMaps.size() == 1) {
            logger_.info("[HandheldScanPipeline] Single disparity map, no fusion needed");
            return disparityMaps[0].clone();
        }

        logger_.info("[HandheldScanPipeline] Fusing " + std::to_string(disparityMaps.size()) +
                    " disparity maps with outlier rejection (sigma=" + std::to_string(outlierSigma) + ")");

        const int height = disparityMaps[0].rows;
        const int width = disparityMaps[0].cols;
        cv::Mat fusedDisparity(height, width, CV_16SC1, cv::Scalar(0));

        int totalPixels = 0;
        int validPixels = 0;

        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++) {
                std::vector<int16_t> disparityValues;
                disparityValues.reserve(disparityMaps.size());

                // Collect disparity values from all frames
                for (const auto& disp : disparityMaps) {
                    int16_t val = disp.at<int16_t>(y, x);
                    if (val > 0) {  // Valid disparity
                        disparityValues.push_back(val);
                    }
                }

                totalPixels++;

                if (disparityValues.empty()) {
                    fusedDisparity.at<int16_t>(y, x) = 0;
                    continue;
                }

                // Calculate mean and std deviation
                float mean = std::accumulate(disparityValues.begin(), disparityValues.end(), 0.0f) / disparityValues.size();
                float variance = 0.0f;
                for (auto val : disparityValues) {
                    float diff = val - mean;
                    variance += diff * diff;
                }
                float stddev = std::sqrt(variance / disparityValues.size());

                // Reject outliers
                std::vector<int16_t> inliers;
                float threshold = outlierSigma * stddev;
                for (auto val : disparityValues) {
                    if (std::abs(val - mean) <= threshold) {
                        inliers.push_back(val);
                    }
                }

                if (inliers.empty()) {
                    fusedDisparity.at<int16_t>(y, x) = 0;
                    continue;
                }

                // Compute median of inliers (robust to remaining outliers)
                std::sort(inliers.begin(), inliers.end());
                int16_t median = inliers[inliers.size() / 2];
                fusedDisparity.at<int16_t>(y, x) = median;
                validPixels++;
            }
        }

        float validPercent = (100.0f * validPixels) / totalPixels;
        logger_.info("[HandheldScanPipeline] Fusion complete: " + std::to_string(validPercent) + "% valid pixels");

        return fusedDisparity;
    }

    cv::Mat generatePointCloud(const cv::Mat& disparityMap, const cv::Mat& colorImage) {
        logger_.info("[HandheldScanPipeline] Generating point cloud from disparity map");

        // Log input disparity statistics
        logger_.info("[HandheldScanPipeline] Input disparity: type=" + std::to_string(disparityMap.type()) +
                    ", size=" + std::to_string(disparityMap.cols) + "x" + std::to_string(disparityMap.rows));

        double minDisp, maxDisp;
        cv::minMaxLoc(disparityMap, &minDisp, &maxDisp);
        logger_.info("[HandheldScanPipeline] Disparity range: [" + std::to_string(minDisp/16.0) +
                    ", " + std::to_string(maxDisp/16.0) + "] pixels (subpixel ×16)");

        // CRITICAL FIX FOR CONE ARTIFACT: Adjust Q matrix for PHYSICAL CROP
        // When useCenterCrop_ is enabled, images are PHYSICALLY CROPPED to 680x420
        // New coordinate system: (0,0) at top-left of crop region
        //
        // Problem: Q matrix from calibration has principal point at (737.69, 364.22) from MATLAB
        // in original 1280x720 coordinate system.
        //
        // After physical crop:
        // - New image size: 680x420
        // - Original principal point (737.69, 364.22) in 1280x720 coords
        //   becomes (421.69, 248.22) in crop coords [subtract (cropLeft_, cropTop_)]
        // - Crop center: (340, 210)
        // - Offset STILL present: ΔX = 81.69 px, ΔY = 38.22 px
        //
        // This offset causes CONE ARTIFACT because reprojectImageTo3D calculates:
        //   X = (u - cx) / (-1/Tx)  where cx should be crop center, not translated principal point
        //   Y = (v - cy)            where cy should be crop center, not translated principal point
        //
        // Solution: Adjust Q(0,3) and Q(1,3) to CROP CENTER (not translated pp!)
        //
        // References:
        // - https://stackoverflow.com/questions/33406177/ (Point cloud from reprojectImageTo3D looks like a cone)
        // - https://answers.opencv.org/question/64155/ (Reprojected points form cone shape)
        // - "When you crop an image, the principal point coordinates change. You need to adjust cx and cy values"

        cv::Mat Q_effective = Q_.clone();

        if (useCenterCrop_) {
            // Original principal point from calibration (in 1280x720 coords)
            double cx_orig = -Q_.at<double>(0, 3);  // 737.69
            double cy_orig = -Q_.at<double>(1, 3);  // 364.22

            // Principal point in cropped coordinate system
            // (translate by subtracting crop offset)
            double cx_in_crop = cx_orig - cropLeft_;  // 737.69 - 316 = 421.69
            double cy_in_crop = cy_orig - cropTop_;   // 364.22 - 116 = 248.22

            // Center of cropped image (680x420)
            double crop_center_x = croppedSize_.width / 2.0;   // 340
            double crop_center_y = croppedSize_.height / 2.0;  // 210

            // Calculate offset (STILL PRESENT after crop!)
            double offset_x = cx_in_crop - crop_center_x;  // 421.69 - 340 = 81.69
            double offset_y = cy_in_crop - crop_center_y;  // 248.22 - 210 = 38.22

            logger_.info("[HandheldScanPipeline] CRITICAL FIX: Adjusting Q matrix for PHYSICAL CROP");
            logger_.info("[HandheldScanPipeline]   Original pp (1280x720 coords): (" +
                        std::to_string(cx_orig) + ", " + std::to_string(cy_orig) + ")");
            logger_.info("[HandheldScanPipeline]   Translated pp (crop coords): (" +
                        std::to_string(cx_in_crop) + ", " + std::to_string(cy_in_crop) + ")");
            logger_.info("[HandheldScanPipeline]   Crop center (target): (" +
                        std::to_string(crop_center_x) + ", " + std::to_string(crop_center_y) + ")");
            logger_.info("[HandheldScanPipeline]   Offset: ΔX = " + std::to_string(offset_x) +
                        " px, ΔY = " + std::to_string(offset_y) + " px");

            // Adjust Q matrix principal point to CROP CENTER (eliminates offset!)
            Q_effective.at<double>(0, 3) = -crop_center_x;  // -340
            Q_effective.at<double>(1, 3) = -crop_center_y;  // -210

            logger_.info("[HandheldScanPipeline]   Q matrix adjusted: cx=" + std::to_string(crop_center_x) +
                        ", cy=" + std::to_string(crop_center_y));
            logger_.info("[HandheldScanPipeline]   Disparity map size: " +
                        std::to_string(disparityMap.cols) + "x" + std::to_string(disparityMap.rows));
            logger_.info("[HandheldScanPipeline]   This FIX should eliminate the CONE ARTIFACT!");
        } else {
            logger_.info("[HandheldScanPipeline] Using Q matrix from calibration file (no crop, no adjustment)");
        }

        logger_.info("[HandheldScanPipeline]   Q(2,2) = " + std::to_string(Q_effective.at<double>(2, 2)));
        logger_.info("[HandheldScanPipeline]   Q(2,3) = " + std::to_string(Q_effective.at<double>(2, 3)) + " (focal length)");
        logger_.info("[HandheldScanPipeline]   Q(3,2) = " + std::to_string(Q_effective.at<double>(3, 2)) + " (-1/baseline)");
        logger_.info("[HandheldScanPipeline]   Q(3,3) = " + std::to_string(Q_effective.at<double>(3, 3)));

        // CRITICAL: Convert disparity from CV_16SC1 (subpixel ×16) to CV_32F (pixel values)
        // reprojectImageTo3D expects disparity in PIXELS, not subpixel format!
        // OpenCV official example: disp.convertTo(floatDisp, CV_32F, 1.0f / 16.0f)
        cv::Mat floatDisparity;
        disparityMap.convertTo(floatDisparity, CV_32F, 1.0 / 16.0);  // Divide by 16!

        logger_.info("[HandheldScanPipeline] Converted disparity: CV_16SC1 (×16) → CV_32F (÷16 = pixel values)");

        // Reproject to 3D using ADJUSTED Q matrix (fixes cone artifact!)
        cv::Mat points3D;
        cv::reprojectImageTo3D(floatDisparity, points3D, Q_effective, true, CV_32F);

        logger_.info("[HandheldScanPipeline] Point cloud generated: " +
                    std::to_string(points3D.cols) + "x" + std::to_string(points3D.rows) + " points");

        // ========== STEP 1: RECTANGULAR BORDER FILTER ==========
        // CRITICAL FIX: Filter RECTANGULAR margin from all 4 edges (not circular!)
        // Image borders are rectangular (680x420), so we need rectangular filtering
        int filteredBorderPoints = 0;
        int totalPointsBeforeFiltering = 0;

        if (enableBorderFilter_ && useCenterCrop_) {
            logger_.info("[HandheldScanPipeline] Applying RECTANGULAR BORDER FILTER...");
            logger_.info("[HandheldScanPipeline]   Image size: " + std::to_string(croppedSize_.width) + "x" +
                        std::to_string(croppedSize_.height));
            logger_.info("[HandheldScanPipeline]   Border margin: " + std::to_string(borderMarginPixels_) + " px from all edges");

            int margin = borderMarginPixels_;
            int validLeft = margin;
            int validRight = croppedSize_.width - margin;
            int validTop = margin;
            int validBottom = croppedSize_.height - margin;

            logger_.info("[HandheldScanPipeline]   Valid region: x=[" + std::to_string(validLeft) + "," +
                        std::to_string(validRight) + "], y=[" + std::to_string(validTop) + "," +
                        std::to_string(validBottom) + "]");

            // Filter all border pixels
            for (int y = 0; y < points3D.rows; y++) {
                for (int x = 0; x < points3D.cols; x++) {
                    cv::Vec3f& pt = points3D.at<cv::Vec3f>(y, x);

                    // Check if point is valid (has valid Z depth)
                    bool isValid = (pt[2] > 0 && pt[2] < 10000);

                    if (isValid) {
                        totalPointsBeforeFiltering++;

                        // Check if point is in border region
                        bool inBorder = (x < validLeft) || (x >= validRight) ||
                                       (y < validTop) || (y >= validBottom);

                        if (inBorder) {
                            // Mark as invalid
                            pt[0] = 0;
                            pt[1] = 0;
                            pt[2] = 10000;  // Invalid Z marker
                            filteredBorderPoints++;
                        }
                    }
                }
            }

            float borderFilterPercent = (totalPointsBeforeFiltering > 0) ?
                (100.0f * filteredBorderPoints / totalPointsBeforeFiltering) : 0.0f;

            logger_.info("[HandheldScanPipeline] Border filter complete:");
            logger_.info("[HandheldScanPipeline]   Total points: " + std::to_string(totalPointsBeforeFiltering));
            logger_.info("[HandheldScanPipeline]   Border points filtered: " + std::to_string(filteredBorderPoints) +
                        " (" + std::to_string(borderFilterPercent) + "%)");
        }

        // ========== STEP 2: STATISTICAL OUTLIER REMOVAL ==========
        // Removes diffuse noise cloud (outlier points in empty space)
        // Method: Check depth consistency with neighbors using local Z variance
        int filteredOutliers = 0;
        int totalValidAfterBorder = 0;

        if (enableStatisticalFilter_) {
            logger_.info("[HandheldScanPipeline] Applying STATISTICAL OUTLIER REMOVAL...");
            logger_.info("[HandheldScanPipeline]   Neighborhood size: " + std::to_string(statisticalFilterK_) + " pixels");
            logger_.info("[HandheldScanPipeline]   Threshold: " + std::to_string(statisticalFilterThreshold_) + " * std_dev");

            int halfK = statisticalFilterK_ / 2;

            // First pass: collect all valid Z depths for global statistics
            std::vector<float> allDepths;
            allDepths.reserve(points3D.rows * points3D.cols);

            for (int y = 0; y < points3D.rows; y++) {
                for (int x = 0; x < points3D.cols; x++) {
                    const cv::Vec3f& pt = points3D.at<cv::Vec3f>(y, x);
                    if (pt[2] > 0 && pt[2] < 10000) {
                        allDepths.push_back(pt[2]);
                        totalValidAfterBorder++;
                    }
                }
            }

            if (allDepths.size() > 100) {
                // Calculate global mean and stddev
                float globalMean = 0;
                for (float z : allDepths) globalMean += z;
                globalMean /= allDepths.size();

                float globalStdDev = 0;
                for (float z : allDepths) {
                    float diff = z - globalMean;
                    globalStdDev += diff * diff;
                }
                globalStdDev = std::sqrt(globalStdDev / allDepths.size());

                logger_.info("[HandheldScanPipeline]   Global depth stats: mean=" +
                            std::to_string(globalMean) + "mm, stddev=" + std::to_string(globalStdDev) + "mm");

                // Second pass: filter outliers based on local neighborhood consistency
                // MULTI-THREADED: Process rows in parallel using 4 threads (RPi5 has 4 Cortex-A76 cores)
                const int numThreads = 4;
                const int rowsPerThread = (points3D.rows + numThreads - 1) / numThreads;
                std::vector<std::thread> threads;
                std::vector<int> threadOutlierCounts(numThreads, 0);

                auto processRows = [&](int threadId, int startRow, int endRow) {
                    int localOutliers = 0;

                    for (int y = startRow; y < endRow && y < points3D.rows; y++) {
                        for (int x = 0; x < points3D.cols; x++) {
                            cv::Vec3f& pt = points3D.at<cv::Vec3f>(y, x);

                            if (pt[2] > 0 && pt[2] < 10000) {
                                // Collect neighbor depths (OPTIMIZED: k=12 instead of 30)
                                std::vector<float> neighborDepths;
                                neighborDepths.reserve(statisticalFilterK_ * statisticalFilterK_);
                                int validNeighbors = 0;

                                for (int dy = -halfK; dy <= halfK; dy++) {
                                    int ny = y + dy;
                                    if (ny < 0 || ny >= points3D.rows) continue;

                                    for (int dx = -halfK; dx <= halfK; dx++) {
                                        int nx = x + dx;
                                        if (nx < 0 || nx >= points3D.cols) continue;

                                        const cv::Vec3f& npt = points3D.at<cv::Vec3f>(ny, nx);
                                        if (npt[2] > 0 && npt[2] < 10000) {
                                            neighborDepths.push_back(npt[2]);
                                            validNeighbors++;
                                        }
                                    }
                                }

                                // Require at least 30% of neighborhood to be valid
                                int minNeighbors = (statisticalFilterK_ * statisticalFilterK_) * 0.3;
                                if (validNeighbors >= minNeighbors) {
                                    // Calculate local mean
                                    float localMean = 0;
                                    for (float z : neighborDepths) localMean += z;
                                    localMean /= neighborDepths.size();

                                    // Calculate local stddev
                                    float localStdDev = 0;
                                    for (float z : neighborDepths) {
                                        float diff = z - localMean;
                                        localStdDev += diff * diff;
                                    }
                                    localStdDev = std::sqrt(localStdDev / neighborDepths.size());

                                    // Check if point is outlier (too far from local mean)
                                    float deviation = std::abs(pt[2] - localMean);
                                    float threshold = statisticalFilterThreshold_ * localStdDev;

                                    // Also check global outliers (extremely far from global mean)
                                    float globalDeviation = std::abs(pt[2] - globalMean);
                                    float globalThreshold = 3.0f * globalStdDev;

                                    if (deviation > threshold || globalDeviation > globalThreshold) {
                                        // Mark as outlier
                                        pt[0] = 0;
                                        pt[1] = 0;
                                        pt[2] = 10000;
                                        localOutliers++;
                                    }
                                } else {
                                    // Insufficient neighbors → likely isolated noise
                                    pt[0] = 0;
                                    pt[1] = 0;
                                    pt[2] = 10000;
                                    localOutliers++;
                                }
                            }
                        }
                    }

                    threadOutlierCounts[threadId] = localOutliers;
                };

                // Launch threads
                for (int t = 0; t < numThreads; t++) {
                    int startRow = t * rowsPerThread;
                    int endRow = startRow + rowsPerThread;
                    threads.emplace_back(processRows, t, startRow, endRow);
                }

                // Wait for all threads to complete
                for (auto& thread : threads) {
                    thread.join();
                }

                // Sum outliers from all threads
                filteredOutliers = 0;
                for (int count : threadOutlierCounts) {
                    filteredOutliers += count;
                }

                float outlierPercent = (totalValidAfterBorder > 0) ?
                    (100.0f * filteredOutliers / totalValidAfterBorder) : 0.0f;

                logger_.info("[HandheldScanPipeline] Statistical outlier removal complete:");
                logger_.info("[HandheldScanPipeline]   Valid points after border filter: " +
                            std::to_string(totalValidAfterBorder));
                logger_.info("[HandheldScanPipeline]   Outliers removed: " + std::to_string(filteredOutliers) +
                            " (" + std::to_string(outlierPercent) + "%)");
                logger_.info("[HandheldScanPipeline]   Final high-quality points: " +
                            std::to_string(totalValidAfterBorder - filteredOutliers));
            } else {
                logger_.warning("[HandheldScanPipeline] Too few valid points for statistical filtering (need >100, have " +
                            std::to_string(allDepths.size()) + ")");
            }
        }

        // Sample Z depths from center region for verification
        const int sampleSize = 100;
        const int centerX = points3D.cols / 2;
        const int centerY = points3D.rows / 2;
        const int sampleRadius = 50;

        std::vector<float> sampleDepths;
        sampleDepths.reserve(sampleSize);

        for (int i = 0; i < sampleSize; i++) {
            int x = centerX + (rand() % (2 * sampleRadius)) - sampleRadius;
            int y = centerY + (rand() % (2 * sampleRadius)) - sampleRadius;

            if (x >= 0 && x < points3D.cols && y >= 0 && y < points3D.rows) {
                cv::Vec3f pt = points3D.at<cv::Vec3f>(y, x);
                float z = pt[2];
                if (z > 0 && z < 10000) {  // Valid range
                    sampleDepths.push_back(z);
                }
            }
        }

        if (!sampleDepths.empty()) {
            std::sort(sampleDepths.begin(), sampleDepths.end());
            float minZ = sampleDepths.front();
            float medianZ = sampleDepths[sampleDepths.size() / 2];
            float maxZ = sampleDepths.back();

            logger_.info("[HandheldScanPipeline] Sample Z depths (100 points from center):");
            logger_.info("[HandheldScanPipeline]   Min: " + std::to_string(minZ) + " mm");
            logger_.info("[HandheldScanPipeline]   Median: " + std::to_string(medianZ) + " mm");
            logger_.info("[HandheldScanPipeline]   Max: " + std::to_string(maxZ) + " mm");
        }

        // Export disparity + 3D data for Python analysis
        if (saveDebugImages_ && !debugDir_.empty()) {
            std::string exportPath = debugDir_ + "/disparity_3d_data.txt";
            std::ofstream exportFile(exportPath);

            if (exportFile.is_open()) {
                exportFile << "# x_pixel y_pixel disparity X Y Z\n";

                for (int y = 0; y < disparityMap.rows; y++) {
                    const int16_t* dispRow = disparityMap.ptr<int16_t>(y);
                    for (int x = 0; x < disparityMap.cols; x++) {
                        int16_t disp16 = dispRow[x];
                        if (disp16 > 0) {
                            cv::Vec3f pt = points3D.at<cv::Vec3f>(y, x);
                            float disparity = disp16 / 16.0f;

                            exportFile << x << " " << y << " "
                                      << disparity << " "
                                      << pt[0] << " " << pt[1] << " " << pt[2] << "\n";
                        }
                    }
                }

                exportFile.close();
                logger_.info("[HandheldScanPipeline] Exported all disparity+3D data to: " + exportPath);
            }
        }

        // Filter point cloud to valid depth range [200mm, 1000mm]
        cv::Mat filteredCloud;
        std::vector<cv::Vec3f> validPoints;

        const float minDepth = 200.0f;   // 200mm minimum
        const float maxDepth = 1000.0f;  // 1000mm maximum

        for (int y = 0; y < points3D.rows; y++) {
            for (int x = 0; x < points3D.cols; x++) {
                cv::Vec3f pt = points3D.at<cv::Vec3f>(y, x);
                float z = pt[2];

                if (z >= minDepth && z <= maxDepth) {
                    validPoints.push_back(pt);
                }
            }
        }

        logger_.info("[HandheldScanPipeline] Filtered to depth range [" +
                    std::to_string(minDepth) + ", " + std::to_string(maxDepth) + "] mm: " +
                    std::to_string(validPoints.size()) + " valid points");

        if (validPoints.empty()) {
            logger_.warning("[HandheldScanPipeline] No valid points in depth range!");
            return cv::Mat();
        }

        // Convert to CV_32FC3 matrix
        filteredCloud = cv::Mat(validPoints.size(), 1, CV_32FC3);
        for (size_t i = 0; i < validPoints.size(); i++) {
            filteredCloud.at<cv::Vec3f>(i, 0) = validPoints[i];
        }

        // FIX: Export PLY file for 3D visualization
        if (saveDebugImages_ && !debugDir_.empty()) {
            std::string plyPath = debugDir_ + "/point_cloud.ply";
            std::ofstream plyFile(plyPath);

            if (plyFile.is_open()) {
                // Write PLY header
                plyFile << "ply\n";
                plyFile << "format ascii 1.0\n";
                plyFile << "element vertex " << validPoints.size() << "\n";
                plyFile << "property float x\n";
                plyFile << "property float y\n";
                plyFile << "property float z\n";
                plyFile << "end_header\n";

                // CRITICAL: Coordinate system conversion
                // OpenCV reprojectImageTo3D uses image coordinates: X=right, Y=down, Z=forward
                // PLY standard uses world coordinates: X=right, Y=up, Z=forward
                // Must INVERT Y axis: y_world = -y_opencv
                // This is a common issue - without inversion, point cloud appears flipped vertically
                for (const auto& pt : validPoints) {
                    float x_meters = pt[0] / 1000.0f;
                    float y_meters = -pt[1] / 1000.0f;  // INVERT Y for correct orientation
                    float z_meters = pt[2] / 1000.0f;

                    plyFile << x_meters << " " << y_meters << " " << z_meters << "\n";
                }

                plyFile.close();
                logger_.info("[HandheldScanPipeline] Exported PLY to: " + plyPath);
                logger_.info("[HandheldScanPipeline]   Coordinate system: X=right, Y=up (inverted), Z=forward");
            } else {
                logger_.warning("[HandheldScanPipeline] Failed to create PLY file: " + plyPath);
            }
        }

        return filteredCloud;
    }
};

// ============================================================================
// PUBLIC API IMPLEMENTATION
// ============================================================================

HandheldScanPipeline::HandheldScanPipeline(std::shared_ptr<camera::CameraSystem> cameraSystem)
    : pImpl(std::make_unique<Impl>(cameraSystem))
{
}

HandheldScanPipeline::~HandheldScanPipeline() = default;

bool HandheldScanPipeline::initialize() {
    // Load default calibration from /unlook_calib/default.yaml
    return pImpl->loadCalibration("");
}

void HandheldScanPipeline::shutdown() {
    pImpl->logger_.info("[HandheldScanPipeline] Shutdown complete");
}

stereo::StereoMatchingParams HandheldScanPipeline::getStereoParams() const {
    // Return current SGMCensus configuration as StereoMatchingParams
    stereo::StereoMatchingParams params;
    params.numDisparities = pImpl->censusConfig_.numDisparities;
    params.minDisparity = pImpl->censusConfig_.minDisparity;
    params.P1 = pImpl->censusConfig_.P1;
    params.P2 = pImpl->censusConfig_.P2;
    return params;
}

void HandheldScanPipeline::setStereoParams(const stereo::StereoMatchingParams& params) {
    pImpl->censusConfig_.numDisparities = params.numDisparities;
    pImpl->censusConfig_.minDisparity = params.minDisparity;
    pImpl->censusConfig_.P1 = params.P1;
    pImpl->censusConfig_.P2 = params.P2;
    pImpl->sgmCensus_ = std::make_unique<stereo::SGMCensus>(pImpl->censusConfig_);
}

float HandheldScanPipeline::calculatePrecision(const std::vector<cv::Mat>& depthMaps) {
    if (depthMaps.empty()) {
        return 0.0f;
    }

    // Stub implementation: return nominal precision
    // TODO: Implement actual variance-based precision calculation
    return 0.1f; // 0.1mm nominal
}

bool HandheldScanPipeline::saveDebugOutput(
    const std::string& debugDir,
    const std::vector<StereoFrame>& frames,
    const std::vector<cv::Mat>& depthMaps,
    const cv::Mat& fusedDepth,
    const cv::Mat& pointCloud)
{
    pImpl->logger_.info("[HandheldScanPipeline] Saving debug output to: " + debugDir);

    // Debug output already handled in processFrames() and generatePointCloud()
    // This is just a placeholder for additional batch exports if needed

    return true;
}

std::vector<cv::Mat> HandheldScanPipeline::processFrames(
    const std::vector<StereoFrame>& frames,
    const stereo::StereoMatchingParams& params,
    ProgressCallback progressCallback)
{
    return pImpl->processFrames(frames, params, progressCallback);
}

cv::Mat HandheldScanPipeline::fuseDisparityMaps(
    const std::vector<cv::Mat>& disparityMaps,
    float outlierSigma)
{
    return pImpl->fuseDisparityMaps(disparityMaps, outlierSigma);
}

cv::Mat HandheldScanPipeline::generatePointCloud(
    const cv::Mat& disparityMap,
    const cv::Mat& colorImage)
{
    return pImpl->generatePointCloud(disparityMap, colorImage);
}

cv::Mat HandheldScanPipeline::getP2() const {
    return pImpl->P2_.clone();
}

void HandheldScanPipeline::setDebugOutput(bool enable, const std::string& outputDir) {
    pImpl->saveDebugImages_ = enable;
    pImpl->debugDir_ = outputDir;

    if (enable) {
        pImpl->logger_.info("[HandheldScanPipeline] Debug output enabled: " + outputDir);
    }
}

} // namespace api
} // namespace unlook
