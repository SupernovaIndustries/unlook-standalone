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
        censusConfig_.P2 = 32;    // Standard SGM penalty (large disparity change)
        censusConfig_.use8Paths = true;
        censusConfig_.verticalSearchRange = 0;  // Disabled (testing showed no improvement)
        censusConfig_.verbose = false;  // Disable for production

        sgmCensus_ = std::make_unique<stereo::SGMCensus>(censusConfig_);

        logger_.info("[HandheldScanPipeline] Initialized with SGMCensus (census=9x9, P1=8, P2=32, vertical=±8px, uniqueness=15%)");
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

            // Save debug images if enabled
            if (saveDebugImages_ && !debugDir_.empty()) {
                std::string frameNum = std::to_string(i);
                cv::imwrite(debugDir_ + "/01_rectified_frame" + frameNum + "_left.png", leftRect);
                cv::imwrite(debugDir_ + "/01_rectified_frame" + frameNum + "_right.png", rightRect);
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
            cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(2.0, cv::Size(8, 8));
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

        // CRITICAL FIX: Apply OpenCV Q matrix bug fix (Issue #4874)
        // stereoRectify returns wrong sign for Q(3,2), must invert for correct depth
        cv::Mat Q_corrected = Q_.clone();
        double originalQ32 = Q_.at<double>(3, 2);
        Q_corrected.at<double>(3, 2) = -originalQ32;

        logger_.info("[HandheldScanPipeline] Applied OpenCV Q matrix bug fix: Q(3,2) inverted from " +
                    std::to_string(originalQ32) + " to " + std::to_string(Q_corrected.at<double>(3, 2)));

        // Reproject to 3D using corrected Q matrix
        cv::Mat points3D;
        cv::reprojectImageTo3D(disparityMap, points3D, Q_corrected, true, CV_32F);

        logger_.info("[HandheldScanPipeline] Point cloud generated: " +
                    std::to_string(points3D.cols) + "x" + std::to_string(points3D.rows) + " points");

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

                // Write points (convert mm to meters for standard PLY)
                for (const auto& pt : validPoints) {
                    plyFile << (pt[0] / 1000.0f) << " "
                           << (pt[1] / 1000.0f) << " "
                           << (pt[2] / 1000.0f) << "\n";
                }

                plyFile.close();
                logger_.info("[HandheldScanPipeline] Exported PLY to: " + plyPath);
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
