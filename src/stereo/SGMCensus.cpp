/**
 * @file SGMCensus.cpp
 * @brief Implementation of production SGM-Census stereo matching
 */

#include <unlook/stereo/SGMCensus.hpp>
#include <iostream>
#include <algorithm>
#include <chrono>
#include <limits>
#include <omp.h>  // OpenMP for multi-threading

namespace unlook {
namespace stereo {

using namespace std::chrono;

SGMCensus::SGMCensus()
    : config_()
{
    // Use default configuration
    if (config_.verbose) {
        std::cout << "[SGM-Census] Initialized with default configuration:" << std::endl;
        std::cout << "  Census window: " << config_.censusWindowSize << "x" << config_.censusWindowSize << std::endl;
        std::cout << "  Disparity range: " << config_.minDisparity << " to " << (config_.minDisparity + config_.numDisparities) << std::endl;
        std::cout << "  SGM penalties: P1=" << config_.P1 << ", P2=" << config_.P2 << std::endl;
        std::cout << "  Paths: " << (config_.use8Paths ? "8" : "4") << " directions" << std::endl;
    }
}

SGMCensus::SGMCensus(const Config& config)
    : config_(config)
{
    if (config_.verbose) {
        std::cout << "[SGM-Census] Initialized:" << std::endl;
        std::cout << "  Census window: " << config_.censusWindowSize << "x" << config_.censusWindowSize << std::endl;
        std::cout << "  Disparity range: " << config_.minDisparity << " to " << (config_.minDisparity + config_.numDisparities) << std::endl;
        std::cout << "  SGM penalties: P1=" << config_.P1 << ", P2=" << config_.P2 << std::endl;
        std::cout << "  Paths: " << (config_.use8Paths ? "8" : "4") << " directions" << std::endl;
    }
}

SGMCensus::Result SGMCensus::compute(
    const cv::Mat& leftGray,
    const cv::Mat& rightGray)
{
    auto totalStart = high_resolution_clock::now();
    Result result;
    result.success = false;

    std::string validationError;
    if (!validateInputs(leftGray, rightGray, validationError)) {
        result.errorMessage = validationError;
        return result;
    }

    const int width = leftGray.cols;
    const int height = leftGray.rows;
    const int D = config_.numDisparities;

    if (config_.verbose) {
        std::cout << "\n[SGM-Census] Processing " << width << "x" << height << " images..." << std::endl;
    }

    try {
        // Census Transform
        auto censusStart = high_resolution_clock::now();
        cv::Mat censusLeft, censusRight;
        computeCensusTransform(leftGray, censusLeft);
        computeCensusTransform(rightGray, censusRight);
        auto censusEnd = high_resolution_clock::now();
        result.censusTimeMs = duration<double, std::milli>(censusEnd - censusStart).count();
        if (config_.verbose) {
            std::cout << "  ✓ Census Transform: " << result.censusTimeMs << " ms" << std::endl;
        }

        // Matching Cost
        auto costStart = high_resolution_clock::now();
        std::vector<uint8_t> costVolume(height * width * D, 255);
        computeMatchingCost(censusLeft, censusRight, costVolume);
        auto costEnd = high_resolution_clock::now();
        result.costTimeMs = duration<double, std::milli>(costEnd - costStart).count();
        if (config_.verbose) {
            std::cout << "  ✓ Matching Cost: " << result.costTimeMs << " ms" << std::endl;
        }

        // SGM Aggregation
        auto sgmStart = high_resolution_clock::now();
        std::vector<uint32_t> aggregatedCost(height * width * D, 0);
        aggregateCostsSGM(costVolume, aggregatedCost, width, height);
        auto sgmEnd = high_resolution_clock::now();
        result.sgmTimeMs = duration<double, std::milli>(sgmEnd - sgmStart).count();
        if (config_.verbose) {
            std::cout << "  ✓ SGM Aggregation: " << result.sgmTimeMs << " ms" << std::endl;
        }

        // WTA Selection
        auto wtaStart = high_resolution_clock::now();
        result.disparity = cv::Mat(height, width, CV_16SC1, cv::Scalar(0));
        selectDisparitiesWTA(aggregatedCost, result.disparity, width, height);
        auto wtaEnd = high_resolution_clock::now();
        result.wtaTimeMs = duration<double, std::milli>(wtaEnd - wtaStart).count();

        // Statistics
        result.validPixels = 0;
        for (int y = 0; y < height; y++) {
            const int16_t* row = result.disparity.ptr<int16_t>(y);
            for (int x = 0; x < width; x++) {
                if (row[x] > 0) result.validPixels++;
            }
        }
        result.validPercent = (100.0 * result.validPixels) / (width * height);

        auto totalEnd = high_resolution_clock::now();
        result.totalTimeMs = duration<double, std::milli>(totalEnd - totalStart).count();
        result.success = true;

        if (config_.verbose) {
            std::cout << "  ✓ WTA Selection: " << result.wtaTimeMs << " ms" << std::endl;
            std::cout << "\n[SGM-Census] Complete: " << result.totalTimeMs << " ms" << std::endl;
            std::cout << "  Valid pixels: " << result.validPercent << "%" << std::endl;
        }

    } catch (const std::exception& e) {
        result.errorMessage = std::string("Exception: ") + e.what();
        result.success = false;
    }

    return result;
}

void SGMCensus::computeCensusTransform(const cv::Mat& image, cv::Mat& census)
{
    const int width = image.cols;
    const int height = image.rows;
    const int radius = config_.censusWindowSize / 2;

    // CRITICAL: Use CV_64FC1 for 64-bit descriptors (7x7 = 48 bits)
    // Adapted from epiception/SGM-Census using unsigned long
    census.create(height, width, CV_64FC1);
    census.setTo(cv::Scalar(0));

    // OPTIMIZATION: Parallelize row processing with OpenMP
    #pragma omp parallel for schedule(dynamic, 16)
    for (int y = radius; y < height - radius; y++) {
        const uint8_t* imgRow = image.ptr<uint8_t>(y);
        uint64_t* censusRow = census.ptr<uint64_t>(y);

        for (int x = radius; x < width - radius; x++) {
            uint64_t descriptor = 0;
            const uint8_t centerVal = imgRow[x];
            int shiftCount = 0;
            const int totalBits = config_.censusWindowSize * config_.censusWindowSize;

            // Iterate in same order as original repo
            for (int dy = -radius; dy <= radius; dy++) {
                const uint8_t* winRow = image.ptr<uint8_t>(y + dy);
                for (int dx = -radius; dx <= radius; dx++) {
                    // Skip center pixel (same as original: shiftCount != window_width*window_height/2)
                    if (shiftCount != totalBits / 2) {
                        descriptor <<= 1;  // Left shift first (like original)
                        uint8_t pixelVal = winRow[x + dx];
                        // bit = 1 if pixel < center (EXACT original logic)
                        if (pixelVal < centerVal) {
                            descriptor |= 1;
                        }
                    }
                    shiftCount++;
                }
            }
            censusRow[x] = descriptor;
        }
    }
}

void SGMCensus::computeMatchingCost(
    const cv::Mat& censusLeft,
    const cv::Mat& censusRight,
    std::vector<uint8_t>& costVolume)
{
    const int width = censusLeft.cols;
    const int height = censusLeft.rows;
    const int D = config_.numDisparities;
    const int verticalRange = config_.verticalSearchRange;  // ±8 pixels for epipolar error

    // OPTIMIZATION: Parallelize row processing with OpenMP
    #pragma omp parallel for schedule(dynamic, 8)
    for (int y = 0; y < height; y++) {
        const uint64_t* leftRow = censusLeft.ptr<uint64_t>(y);

        for (int x = 0; x < width; x++) {
            uint64_t leftDesc = leftRow[x];

            for (int d = 0; d < D; d++) {
                int xr = x - d;
                if (xr < 0 || xr >= width) {
                    costVolume[y * width * D + x * D + d] = 255;
                    continue;
                }

                // CRITICAL FIX: Search in vertical window ±verticalRange to compensate for epipolar error
                // With max_epipolar_error=7.3 pixels, VCSEL dots may be shifted vertically
                // Find MINIMUM cost across all vertical offsets
                int minCost = 255;
                for (int dy = -verticalRange; dy <= verticalRange; dy++) {
                    int yr = y + dy;
                    if (yr < 0 || yr >= height) {
                        continue;  // Skip out-of-bounds
                    }

                    const uint64_t* rightRow = censusRight.ptr<uint64_t>(yr);
                    uint64_t rightDesc = rightRow[xr];

                    // Hamming distance between Census descriptors
                    int hamming = hammingWeight(leftDesc ^ rightDesc);
                    minCost = std::min(minCost, hamming);
                }

                // Store minimum cost found across vertical search window
                costVolume[y * width * D + x * D + d] = std::min(minCost, 255);
            }
        }
    }
}

void SGMCensus::aggregateCostsSGM(
    const std::vector<uint8_t>& costVolume,
    std::vector<uint32_t>& aggregatedCost,
    int width, int height)
{
    const int D = config_.numDisparities;
    const int numPaths = config_.use8Paths ? 8 : 4;

    std::vector<std::vector<uint16_t>> pathCosts(numPaths);
    for (int i = 0; i < numPaths; i++) {
        pathCosts[i].resize(height * width * D, 0);
    }

    std::vector<std::pair<int, int>> directions;
    directions.push_back({1, 0});   // L→R
    directions.push_back({-1, 0});  // R→L
    directions.push_back({0, 1});   // T→B
    directions.push_back({0, -1});  // B→T

    if (config_.use8Paths) {
        directions.push_back({1, 1});    // TL→BR
        directions.push_back({-1, -1});  // BR→TL
        directions.push_back({1, -1});   // BL→TR
        directions.push_back({-1, 1});   // TR→BL
    }

    // OPTIMIZATION: Parallelize path processing (paths are independent)
    #pragma omp parallel for schedule(dynamic)
    for (int pathIdx = 0; pathIdx < numPaths; pathIdx++) {
        int dirX = directions[pathIdx].first;
        int dirY = directions[pathIdx].second;
        processSGMPath(costVolume, pathCosts[pathIdx], dirX, dirY, width, height);
    }

    // OPTIMIZATION: Parallelize aggregation across volume
    #pragma omp parallel for
    for (int idx = 0; idx < height * width * D; idx++) {
        for (int pathIdx = 0; pathIdx < numPaths; pathIdx++) {
            aggregatedCost[idx] += pathCosts[pathIdx][idx];
        }
    }
}

void SGMCensus::processSGMPath(
    const std::vector<uint8_t>& costVolume,
    std::vector<uint16_t>& pathCost,
    int dirX, int dirY,
    int width, int height)
{
    const int D = config_.numDisparities;
    const int P1 = config_.P1;
    const int P2 = config_.P2;

    int startX = (dirX > 0) ? 0 : (dirX < 0) ? width - 1 : 0;
    int endX = (dirX > 0) ? width : (dirX < 0) ? -1 : width;
    int stepX = (dirX == 0) ? 1 : dirX;

    int startY = (dirY > 0) ? 0 : (dirY < 0) ? height - 1 : 0;
    int endY = (dirY > 0) ? height : (dirY < 0) ? -1 : height;
    int stepY = (dirY == 0) ? 1 : dirY;

    for (int y = startY; y != endY; y += stepY) {
        for (int x = startX; x != endX; x += stepX) {
            int prevX = x - dirX;
            int prevY = y - dirY;
            bool isBoundary = (prevX < 0 || prevX >= width || prevY < 0 || prevY >= height);

            if (isBoundary) {
                for (int d = 0; d < D; d++) {
                    int idx = y * width * D + x * D + d;
                    pathCost[idx] = costVolume[idx];
                }
            } else {
                int prevIdx = prevY * width * D + prevX * D;
                uint16_t minPrevAll = UINT16_MAX;
                for (int d = 0; d < D; d++) {
                    minPrevAll = std::min(minPrevAll, pathCost[prevIdx + d]);
                }

                for (int d = 0; d < D; d++) {
                    int idx = y * width * D + x * D + d;
                    uint8_t matchCost = costVolume[idx];

                    uint16_t costSame = pathCost[prevIdx + d];
                    uint16_t costMinus = (d > 0) ? pathCost[prevIdx + d - 1] + P1 : UINT16_MAX;
                    uint16_t costPlus = (d < D - 1) ? pathCost[prevIdx + d + 1] + P1 : UINT16_MAX;
                    uint16_t costNeighbor = std::min(costMinus, costPlus);
                    uint16_t costOther = minPrevAll + P2;

                    uint16_t minPath = std::min({costSame, costNeighbor, costOther});
                    uint16_t aggregated = matchCost + minPath - minPrevAll;
                    pathCost[idx] = aggregated;
                }
            }
        }
    }
}

void SGMCensus::selectDisparitiesWTA(
    const std::vector<uint32_t>& aggregatedCost,
    cv::Mat& disparity,
    int width, int height)
{
    const int D = config_.numDisparities;
    const bool checkUniqueness = config_.enableUniquenessCheck;
    const int uniquenessRatio = config_.uniquenessRatio;

    // OPTIMIZATION: Parallelize row processing with OpenMP
    #pragma omp parallel for schedule(dynamic, 8)
    for (int y = 0; y < height; y++) {
        int16_t* dispRow = disparity.ptr<int16_t>(y);
        for (int x = 0; x < width; x++) {
            uint32_t minCost = UINT32_MAX;
            uint32_t secondMinCost = UINT32_MAX;
            int bestD = 0;

            // Find best and second-best matches
            for (int d = 0; d < D; d++) {
                int idx = y * width * D + x * D + d;
                uint32_t cost = aggregatedCost[idx];

                if (cost < minCost) {
                    secondMinCost = minCost;
                    minCost = cost;
                    bestD = d;
                } else if (cost < secondMinCost) {
                    secondMinCost = cost;
                }
            }

            // UNIQUENESS CHECK (like Luxonis HIGH_ACCURACY mode)
            // If second-best match is too close to best match → ambiguous → invalidate
            bool isUnique = true;
            if (checkUniqueness && minCost > 0) {
                // Calculate uniqueness margin: (second - first) / first * 100
                int margin = ((secondMinCost - minCost) * 100) / (minCost + 1);
                if (margin < uniquenessRatio) {
                    isUnique = false;  // Match is ambiguous
                }
            }

            if (!isUnique) {
                dispRow[x] = 0;  // Invalid/ambiguous
                continue;
            }

            // CRITICAL: SUBPIXEL INTERPOLATION using parabola fitting
            // References:
            // - Luxonis OAK: 3 fractional bits = 8 subpixel steps
            // - OpenCV SGBM: parabola fitting on cost volume
            // - Papers: significantly improves depth accuracy at long range
            //
            // Formula: offset = (C[d-1] - C[d+1]) / (2*(C[d-1] - 2*C[d] + C[d+1]))
            // This fits a parabola through 3 cost values and finds the minimum

            float subpixelOffset = 0.0f;

            if (bestD > 0 && bestD < D - 1) {
                // Get costs around best match
                int baseIdx = y * width * D + x * D;
                uint32_t C_prev = aggregatedCost[baseIdx + bestD - 1];
                uint32_t C_curr = aggregatedCost[baseIdx + bestD];
                uint32_t C_next = aggregatedCost[baseIdx + bestD + 1];

                // Parabola interpolation formula
                float numerator = static_cast<float>(C_prev - C_next);
                float denominator = 2.0f * static_cast<float>(C_prev - 2*C_curr + C_next);

                // Avoid division by zero and numerical instability
                if (std::abs(denominator) > 1e-6f) {
                    subpixelOffset = numerator / denominator;
                    // Clamp to valid range [-1, 1] (shouldn't exceed but be safe)
                    subpixelOffset = std::max(-1.0f, std::min(1.0f, subpixelOffset));
                }
            }

            // Final disparity with subpixel precision
            // OpenCV format: integer_disparity * 16 + fractional_bits
            // This gives us ~1500 depth levels instead of 96
            float disparitySubpixel = (static_cast<float>(bestD) + subpixelOffset) * 16.0f;
            dispRow[x] = static_cast<int16_t>(std::round(disparitySubpixel));
        }
    }
}

bool SGMCensus::validateInputs(
    const cv::Mat& left,
    const cv::Mat& right,
    std::string& error)
{
    if (left.empty() || right.empty()) {
        error = "Empty input images";
        return false;
    }
    if (left.size() != right.size()) {
        error = "Image size mismatch";
        return false;
    }
    if (left.type() != CV_8UC1 || right.type() != CV_8UC1) {
        error = "Images must be grayscale (CV_8UC1)";
        return false;
    }
    return true;
}

} // namespace stereo
} // namespace unlook
