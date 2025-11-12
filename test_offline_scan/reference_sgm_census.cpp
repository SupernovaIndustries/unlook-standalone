/**
 * @file reference_sgm_census.cpp
 * @brief Implementation of reference SGM-Census stereo matching
 */

#include "reference_sgm_census.hpp"
#include <iostream>
#include <algorithm>
#include <chrono>
#include <limits>

namespace unlook {
namespace stereo {
namespace reference {

using namespace std::chrono;

ReferenceSGMCensus::ReferenceSGMCensus(const Config& config)
    : config_(config)
{
    if (config_.verbose) {
        std::cout << "[Reference SGM-Census] Initialized:" << std::endl;
        std::cout << "  Census window: " << config_.censusWindowSize << "x" << config_.censusWindowSize << std::endl;
        std::cout << "  Disparity range: " << config_.minDisparity << " to " << (config_.minDisparity + config_.numDisparities) << std::endl;
        std::cout << "  SGM penalties: P1=" << config_.P1 << ", P2=" << config_.P2 << std::endl;
        std::cout << "  Paths: " << (config_.use8Paths ? "8" : "4") << " directions" << std::endl;
    }
}

ReferenceSGMCensus::Result ReferenceSGMCensus::compute(
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

void ReferenceSGMCensus::computeCensusTransform(const cv::Mat& image, cv::Mat& census)
{
    const int width = image.cols;
    const int height = image.rows;
    const int radius = config_.censusWindowSize / 2;

    // Use CV_32SC1 for 32-bit signed integer (can store up to 32-bit descriptors)
    census.create(height, width, CV_32SC1);
    census.setTo(cv::Scalar(0));

    for (int y = radius; y < height - radius; y++) {
        const uint8_t* imgRow = image.ptr<uint8_t>(y);
        int32_t* censusRow = census.ptr<int32_t>(y);

        for (int x = radius; x < width - radius; x++) {
            uint32_t descriptor = 0;
            const uint8_t centerVal = imgRow[x];
            int bitPos = 0;

            for (int dy = -radius; dy <= radius; dy++) {
                const uint8_t* winRow = image.ptr<uint8_t>(y + dy);
                for (int dx = -radius; dx <= radius; dx++) {
                    if (dy == 0 && dx == 0) continue;
                    uint8_t pixelVal = winRow[x + dx];
                    bool bit = (pixelVal < centerVal);  // CRITICAL: < not >
                    descriptor |= (uint32_t(bit) << bitPos);
                    bitPos++;
                }
            }
            censusRow[x] = descriptor;
        }
    }
}

void ReferenceSGMCensus::computeMatchingCost(
    const cv::Mat& censusLeft,
    const cv::Mat& censusRight,
    std::vector<uint8_t>& costVolume)
{
    const int width = censusLeft.cols;
    const int height = censusLeft.rows;
    const int D = config_.numDisparities;

    for (int y = 0; y < height; y++) {
        const int32_t* leftRow = censusLeft.ptr<int32_t>(y);
        const int32_t* rightRow = censusRight.ptr<int32_t>(y);

        for (int x = 0; x < width; x++) {
            uint32_t leftDesc = leftRow[x];

            for (int d = 0; d < D; d++) {
                int xr = x - d;
                if (xr < 0 || xr >= width) {
                    costVolume[y * width * D + x * D + d] = 255;
                    continue;
                }

                uint32_t rightDesc = rightRow[xr];
                int hamming = hammingWeight(leftDesc ^ rightDesc);
                // Use RAW hamming distance like original repo (no *10 scaling)
                int cost = std::min(hamming, 255);
                costVolume[y * width * D + x * D + d] = cost;
            }
        }
    }
}

void ReferenceSGMCensus::aggregateCostsSGM(
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

    for (int pathIdx = 0; pathIdx < numPaths; pathIdx++) {
        int dirX = directions[pathIdx].first;
        int dirY = directions[pathIdx].second;
        processSGMPath(costVolume, pathCosts[pathIdx], dirX, dirY, width, height);
    }

    for (int idx = 0; idx < height * width * D; idx++) {
        for (int pathIdx = 0; pathIdx < numPaths; pathIdx++) {
            aggregatedCost[idx] += pathCosts[pathIdx][idx];
        }
    }
}

void ReferenceSGMCensus::processSGMPath(
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

void ReferenceSGMCensus::selectDisparitiesWTA(
    const std::vector<uint32_t>& aggregatedCost,
    cv::Mat& disparity,
    int width, int height)
{
    const int D = config_.numDisparities;

    for (int y = 0; y < height; y++) {
        int16_t* dispRow = disparity.ptr<int16_t>(y);
        for (int x = 0; x < width; x++) {
            uint32_t minCost = UINT32_MAX;
            int bestD = 0;

            for (int d = 0; d < D; d++) {
                int idx = y * width * D + x * D + d;
                if (aggregatedCost[idx] < minCost) {
                    minCost = aggregatedCost[idx];
                    bestD = d;
                }
            }
            dispRow[x] = static_cast<int16_t>(bestD * 16);
        }
    }
}

bool ReferenceSGMCensus::validateInputs(
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

} // namespace reference
} // namespace stereo
} // namespace unlook
