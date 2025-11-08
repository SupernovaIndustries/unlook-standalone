/**
 * @file DebugOutputManager.cpp
 * @brief Implementation of debug output management
 *
 * Based on MEGA_PROMPT_BACKEND_REWRITE.md specification.
 */

#include "unlook/stereo/DebugOutputManager.hpp"
#include "unlook/core/Logger.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <ctime>

namespace unlook {
namespace stereo {

// ========== CONSTRUCTOR / DESTRUCTOR ==========

DebugOutputManager::DebugOutputManager(core::Logger* logger)
    : logger_(logger)
{
}

DebugOutputManager::~DebugOutputManager() = default;

// ========== CONFIGURATION ==========

void DebugOutputManager::setConfig(const Config& config) {
    config_ = config;
}

// ========== SESSION MANAGEMENT ==========

std::string DebugOutputManager::generateSessionName() {
    auto now = std::chrono::system_clock::now();
    auto timeT = std::chrono::system_clock::to_time_t(now);

    std::ostringstream oss;
    oss << "scan_" << std::put_time(std::localtime(&timeT), "%Y%m%d_%H%M%S");
    return oss.str();
}

std::string DebugOutputManager::createSession(const std::string& sessionName) {
    if (!isEnabled()) {
        return "";
    }

    std::string name = sessionName.empty() ? generateSessionName() : sessionName;
    currentSessionDir_ = config_.outputDir + "/" + name;

    try {
        std::filesystem::create_directories(currentSessionDir_);

        if (logger_) {
            logger_->info("Debug session created: " + currentSessionDir_);
        }

        return currentSessionDir_;
    } catch (const std::filesystem::filesystem_error& e) {
        if (logger_) {
            logger_->error("Failed to create debug session directory: " +
                          std::string(e.what()));
        }
        currentSessionDir_.clear();
        return "";
    }
}

bool DebugOutputManager::ensureSessionDir() {
    if (currentSessionDir_.empty()) {
        if (logger_) {
            logger_->warn("No debug session created, creating default session");
        }
        createSession();
    }

    return !currentSessionDir_.empty();
}

// ========== INPUT IMAGES ==========

void DebugOutputManager::saveInputImages(
    const cv::Mat& leftInput,
    const cv::Mat& rightInput)
{
    if (!isEnabled() || !config_.saveInputImages) {
        return;
    }

    if (!ensureSessionDir()) return;

    try {
        std::string leftPath = currentSessionDir_ + "/00_input_left.png";
        std::string rightPath = currentSessionDir_ + "/00_input_right.png";

        cv::imwrite(leftPath, leftInput);
        cv::imwrite(rightPath, rightInput);

        if (logger_) {
            logger_->info("Saved input images: " + leftPath);
            logger_->info("  Size: " + std::to_string(leftInput.cols) + "x" +
                         std::to_string(leftInput.rows));
        }
    } catch (const cv::Exception& e) {
        if (logger_) {
            logger_->error("Failed to save input images: " + std::string(e.what()));
        }
    }
}

// ========== RECTIFIED IMAGES ==========

void DebugOutputManager::saveRectifiedImages(
    const cv::Mat& leftRect,
    const cv::Mat& rightRect)
{
    if (!isEnabled() || !config_.saveRectified) {
        return;
    }

    if (!ensureSessionDir()) return;

    try {
        // Save individual rectified images
        std::string leftPath = currentSessionDir_ + "/01_rectified_left.png";
        std::string rightPath = currentSessionDir_ + "/01_rectified_right.png";

        cv::imwrite(leftPath, leftRect);
        cv::imwrite(rightPath, rightRect);

        // Save epipolar check visualization
        if (config_.saveEpipolarCheck) {
            cv::Mat epipolarViz = drawEpipolarLines(leftRect, rightRect);
            std::string epipolarPath = currentSessionDir_ + "/01_epipolar_check.png";
            cv::imwrite(epipolarPath, epipolarViz);
        }

        if (logger_) {
            logger_->info("Saved rectified images: " + leftPath);
        }
    } catch (const cv::Exception& e) {
        if (logger_) {
            logger_->error("Failed to save rectified images: " + std::string(e.what()));
        }
    }
}

// ========== RAW DISPARITY ==========

void DebugOutputManager::saveRawDisparity(const cv::Mat& rawDisparity) {
    if (!isEnabled() || !config_.saveRawDisparity) {
        return;
    }

    if (!ensureSessionDir()) return;

    try {
        // Save raw TIFF (16-bit)
        if (config_.saveRawTiff) {
            std::string tiffPath = currentSessionDir_ + "/02_disparity_raw.tiff";
            cv::imwrite(tiffPath, rawDisparity);
        }

        // Save colormap visualization
        if (config_.saveColorMapped) {
            cv::Mat colormap = applyColorMap(rawDisparity, cv::COLORMAP_JET);
            std::string colorPath = currentSessionDir_ + "/02_disparity_raw_vis.png";
            cv::imwrite(colorPath, colormap);
        }

        if (logger_) {
            double minVal, maxVal;
            cv::minMaxLoc(rawDisparity, &minVal, &maxVal);
            logger_->info("Saved raw disparity (range: " +
                         std::to_string(minVal) + " - " + std::to_string(maxVal) + ")");
        }
    } catch (const cv::Exception& e) {
        if (logger_) {
            logger_->error("Failed to save raw disparity: " + std::string(e.what()));
        }
    }
}

// ========== FILTERED DISPARITY ==========

void DebugOutputManager::saveFilteredDisparity(const cv::Mat& filteredDisparity) {
    if (!isEnabled() || !config_.saveFilteredDisparity) {
        return;
    }

    if (!ensureSessionDir()) return;

    try {
        // Save raw TIFF
        if (config_.saveRawTiff) {
            std::string tiffPath = currentSessionDir_ + "/03_disparity_filtered.tiff";
            cv::imwrite(tiffPath, filteredDisparity);
        }

        // Save colormap visualization
        if (config_.saveColorMapped) {
            cv::Mat colormap = applyColorMap(filteredDisparity, cv::COLORMAP_JET);
            std::string colorPath = currentSessionDir_ + "/03_disparity_filtered_vis.png";
            cv::imwrite(colorPath, colormap);
        }

        if (logger_) {
            logger_->info("Saved filtered disparity");
        }
    } catch (const cv::Exception& e) {
        if (logger_) {
            logger_->error("Failed to save filtered disparity: " + std::string(e.what()));
        }
    }
}

// ========== CONFIDENCE MAP ==========

void DebugOutputManager::saveConfidenceMap(const cv::Mat& confidenceMap) {
    if (!isEnabled() || !config_.saveConfidenceMap) {
        return;
    }

    if (!ensureSessionDir()) return;

    try {
        std::string path = currentSessionDir_ + "/04_confidence_map.png";
        cv::imwrite(path, confidenceMap);

        if (logger_) {
            logger_->info("Saved confidence map: " + path);
        }
    } catch (const cv::Exception& e) {
        if (logger_) {
            logger_->error("Failed to save confidence map: " + std::string(e.what()));
        }
    }
}

// ========== DEPTH MAP ==========

void DebugOutputManager::saveDepthMap(const cv::Mat& depthMap) {
    if (!isEnabled() || !config_.saveDepthMap) {
        return;
    }

    if (!ensureSessionDir()) return;

    try {
        // Save raw TIFF (32-bit float)
        if (config_.saveRawTiff) {
            std::string tiffPath = currentSessionDir_ + "/05_depth_map.tiff";
            cv::imwrite(tiffPath, depthMap);
        }

        // Save colormap visualization (TURBO for depth)
        if (config_.saveColorMapped) {
            cv::Mat colormap = applyColorMap(depthMap, cv::COLORMAP_TURBO);
            std::string colorPath = currentSessionDir_ + "/05_depth_map_vis.png";
            cv::imwrite(colorPath, colormap);
        }

        if (logger_) {
            double minVal, maxVal;
            cv::minMaxLoc(depthMap, &minVal, &maxVal);
            logger_->info("Saved depth map (range: " +
                         std::to_string(minVal) + " - " + std::to_string(maxVal) + " mm)");
        }
    } catch (const cv::Exception& e) {
        if (logger_) {
            logger_->error("Failed to save depth map: " + std::string(e.what()));
        }
    }
}

// ========== DEPTH HISTOGRAM ==========

void DebugOutputManager::saveDepthHistogram(const cv::Mat& depthMap) {
    if (!isEnabled() || !config_.saveDepthHistogram) {
        return;
    }

    if (!ensureSessionDir()) return;

    try {
        cv::Mat histogram = createDepthHistogram(depthMap);
        std::string path = currentSessionDir_ + "/06_depth_histogram.png";
        cv::imwrite(path, histogram);

        if (logger_) {
            logger_->info("Saved depth histogram: " + path);
        }
    } catch (const cv::Exception& e) {
        if (logger_) {
            logger_->error("Failed to save depth histogram: " + std::string(e.what()));
        }
    }
}

// ========== POINT CLOUD ==========

void DebugOutputManager::savePointCloud(const std::string& plyPath) {
    if (!isEnabled() || !config_.savePointCloud) {
        return;
    }

    // Copy PLY to debug session directory
    if (!ensureSessionDir()) return;

    try {
        std::string destPath = currentSessionDir_ + "/07_pointcloud.ply";
        std::filesystem::copy_file(
            plyPath,
            destPath,
            std::filesystem::copy_options::overwrite_existing);

        if (logger_) {
            logger_->info("Saved point cloud: " + destPath);
        }
    } catch (const std::filesystem::filesystem_error& e) {
        if (logger_) {
            logger_->error("Failed to save point cloud: " + std::string(e.what()));
        }
    }
}

// ========== PROCESSING REPORT ==========

void DebugOutputManager::saveProcessingReport(const ProcessingMetrics& metrics) {
    if (!isEnabled() || !config_.saveProcessingReport) {
        return;
    }

    if (!ensureSessionDir()) return;

    try {
        std::string path = currentSessionDir_ + "/processing_report.txt";
        std::ofstream report(path);

        report << "═══════════════════════════════════════════════════════════\n";
        report << "          STEREO PROCESSING REPORT\n";
        report << "═══════════════════════════════════════════════════════════\n\n";

        // Input information
        report << "INPUT:\n";
        report << "  Image size:        " << metrics.inputSize.width << "x"
               << metrics.inputSize.height << "\n";
        report << "  Calibration size:  " << metrics.calibSize.width << "x"
               << metrics.calibSize.height << "\n";
        report << "  Size match:        "
               << (metrics.inputSize == metrics.calibSize ? "✓ YES" : "✗ NO") << "\n\n";

        // Quality metrics
        report << "QUALITY:\n";
        report << "  Valid pixels:      " << std::fixed << std::setprecision(1)
               << metrics.validPixelPercentage << "%\n";
        report << "  Mean depth:        " << std::setprecision(2)
               << metrics.meanDepthMM << " mm\n";
        report << "  Depth std dev:     " << metrics.depthStdDevMM << " mm\n";
        report << "  Depth range:       " << metrics.minDepthMM << " - "
               << metrics.maxDepthMM << " mm\n\n";

        // Performance metrics
        report << "PERFORMANCE:\n";
        report << "  Rectification:     " << metrics.rectificationTime.count() << " ms\n";
        report << "  Disparity:         " << metrics.disparityTime.count() << " ms\n";
        report << "  Filtering:         " << metrics.filteringTime.count() << " ms\n";
        report << "  Point cloud:       " << metrics.pointCloudTime.count() << " ms\n";
        report << "  ────────────────────────────────────\n";
        report << "  TOTAL:             " << metrics.totalTime.count() << " ms\n";
        report << "  FPS:               " << std::setprecision(1)
               << (1000.0 / metrics.totalTime.count()) << "\n\n";

        // GPU information
        report << "GPU:\n";
        report << "  GPU used:          " << (metrics.gpuUsed ? "YES" : "NO") << "\n";
        if (metrics.gpuUsed) {
            report << "  GPU utilization:   " << metrics.gpuUtilization << "%\n";
            report << "  GPU memory:        " << metrics.gpuMemoryMB << " MB\n";
        }
        report << "\n";

        report << "═══════════════════════════════════════════════════════════\n";
        report.close();

        if (logger_) {
            logger_->info("Processing report saved: " + path);
        }
    } catch (const std::exception& e) {
        if (logger_) {
            logger_->error("Failed to save processing report: " + std::string(e.what()));
        }
    }
}

// ========== UTILITY METHODS ==========

cv::Mat DebugOutputManager::drawEpipolarLines(
    const cv::Mat& leftRect,
    const cv::Mat& rightRect,
    int lineSpacing)
{
    // Convert to color if grayscale
    cv::Mat leftColor, rightColor;
    if (leftRect.channels() == 1) {
        cv::cvtColor(leftRect, leftColor, cv::COLOR_GRAY2BGR);
    } else {
        leftColor = leftRect.clone();
    }

    if (rightRect.channels() == 1) {
        cv::cvtColor(rightRect, rightColor, cv::COLOR_GRAY2BGR);
    } else {
        rightColor = rightRect.clone();
    }

    // Draw horizontal lines
    const cv::Scalar lineColor(0, 255, 0); // Green
    const int thickness = 1;

    for (int y = 0; y < leftColor.rows; y += lineSpacing) {
        cv::line(leftColor,
                cv::Point(0, y),
                cv::Point(leftColor.cols - 1, y),
                lineColor,
                thickness);

        cv::line(rightColor,
                cv::Point(0, y),
                cv::Point(rightColor.cols - 1, y),
                lineColor,
                thickness);
    }

    // Combine horizontally
    cv::Mat combined;
    cv::hconcat(leftColor, rightColor, combined);

    // Add text
    cv::putText(combined,
                "Epipolar Lines Check - Horizontal = Correct Rectification",
                cv::Point(10, 30),
                cv::FONT_HERSHEY_SIMPLEX,
                0.6,
                cv::Scalar(255, 255, 0),
                2);

    return combined;
}

cv::Mat DebugOutputManager::createDepthHistogram(
    const cv::Mat& depthMap,
    int histHeight,
    int histWidth)
{
    // Filter out invalid values (0 or very large)
    cv::Mat validDepth;
    cv::Mat mask = (depthMap > 0) & (depthMap < 10000); // < 10m
    depthMap.copyTo(validDepth, mask);

    if (cv::countNonZero(mask) == 0) {
        // No valid data, return black image
        return cv::Mat::zeros(histHeight, histWidth, CV_8UC3);
    }

    // Calculate histogram
    double minVal, maxVal;
    cv::minMaxLoc(validDepth, &minVal, &maxVal, nullptr, nullptr, mask);

    const int histSize = 256;
    float range[] = {static_cast<float>(minVal), static_cast<float>(maxVal)};
    const float* histRange = {range};

    cv::Mat hist;
    cv::calcHist(&validDepth, 1, 0, mask, hist, 1, &histSize, &histRange);

    // Normalize
    cv::normalize(hist, hist, 0, histHeight * 0.9, cv::NORM_MINMAX);

    // Draw histogram
    cv::Mat histImage = cv::Mat::zeros(histHeight, histWidth, CV_8UC3);
    int binWidth = std::max(1, histWidth / histSize);

    for (int i = 0; i < histSize; i++) {
        int binHeight = static_cast<int>(hist.at<float>(i));
        cv::rectangle(histImage,
                     cv::Point(i * binWidth, histHeight),
                     cv::Point((i + 1) * binWidth, histHeight - binHeight),
                     cv::Scalar(255, 200, 100),
                     cv::FILLED);
    }

    // Add labels
    cv::putText(histImage,
                "Depth Histogram (mm)",
                cv::Point(10, 30),
                cv::FONT_HERSHEY_SIMPLEX,
                0.6,
                cv::Scalar(255, 255, 255),
                2);

    std::ostringstream ossRange;
    ossRange << std::fixed << std::setprecision(0)
             << "Range: " << minVal << " - " << maxVal << " mm";
    cv::putText(histImage,
                ossRange.str(),
                cv::Point(10, histHeight - 10),
                cv::FONT_HERSHEY_SIMPLEX,
                0.5,
                cv::Scalar(255, 255, 255),
                1);

    return histImage;
}

cv::Mat DebugOutputManager::applyColorMap(const cv::Mat& input, int colormapType) {
    if (input.empty()) {
        return cv::Mat();
    }

    cv::Mat normalized;
    cv::normalize(input, normalized, 0, 255, cv::NORM_MINMAX, CV_8U);

    cv::Mat colormap;
    cv::applyColorMap(normalized, colormap, colormapType);

    return colormap;
}

} // namespace stereo
} // namespace unlook
