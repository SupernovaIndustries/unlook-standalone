/**
 * @file test_census_diagnostic.cpp
 * @brief Diagnostic tool to visualize Census descriptors and cost volumes
 *
 * This tool helps debug AD-Census by:
 * 1. Computing Census on left/right images
 * 2. Visualizing Census bit patterns
 * 3. Saving cost volume slices at different disparities
 * 4. Comparing with/without CLAHE preprocessing
 */

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <fstream>

// Forward declare Census functions (simplified versions for testing)
void computeCensusSimple(const cv::Mat& image, cv::Mat& census, int radius = 4);
void computeHammingCostSimple(const cv::Mat& censusLeft, const cv::Mat& censusRight,
                               cv::Mat& costVolume, int minDisp, int numDisp);
void saveCostSlice(const cv::Mat& costVolume, const std::string& filename, int dispSlice);

int main(int argc, char** argv) {
    if (argc != 3) {
        std::cerr << "Usage: " << argv[0] << " <left_rect.png> <right_rect.png>" << std::endl;
        return 1;
    }

    // Load rectified images
    cv::Mat leftRect = cv::imread(argv[1], cv::IMREAD_GRAYSCALE);
    cv::Mat rightRect = cv::imread(argv[2], cv::IMREAD_GRAYSCALE);

    if (leftRect.empty() || rightRect.empty()) {
        std::cerr << "ERROR: Could not load images!" << std::endl;
        return 1;
    }

    std::cout << "=== CENSUS DIAGNOSTIC TOOL ===" << std::endl;
    std::cout << "Image size: " << leftRect.size() << std::endl;

    // Analyze input statistics
    cv::Scalar meanL = cv::mean(leftRect);
    cv::Scalar meanR = cv::mean(rightRect);
    cv::Mat stdL, stdR;
    cv::meanStdDev(leftRect, cv::Scalar(), stdL);
    cv::meanStdDev(rightRect, cv::Scalar(), stdR);

    std::cout << "\n--- Input Statistics ---" << std::endl;
    std::cout << "Left:  mean=" << meanL[0] << ", std=" << stdL.at<double>(0) << std::endl;
    std::cout << "Right: mean=" << meanR[0] << ", std=" << stdR.at<double>(0) << std::endl;

    // Apply CLAHE for better contrast
    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
    cv::Mat leftEnh, rightEnh;
    clahe->apply(leftRect, leftEnh);
    clahe->apply(rightRect, rightEnh);

    cv::Scalar meanLE = cv::mean(leftEnh);
    cv::meanStdDev(leftEnh, cv::Scalar(), stdL);
    std::cout << "\n--- After CLAHE ---" << std::endl;
    std::cout << "Left enhanced: mean=" << meanLE[0] << ", std=" << stdL.at<double>(0) << std::endl;

    cv::imwrite("debug_left_enhanced.png", leftEnh);
    cv::imwrite("debug_right_enhanced.png", rightEnh);

    // Compute Census transforms
    std::cout << "\n--- Computing Census Transforms ---" << std::endl;
    cv::Mat censusLeft, censusRight;
    computeCensusSimple(leftEnh, censusLeft, 4);
    computeCensusSimple(rightEnh, censusRight, 4);
    std::cout << "Census computed: " << censusLeft.size() << std::endl;

    // Visualize Census patterns at center
    int cy = censusLeft.rows / 2;
    int cx = censusLeft.cols / 2;
    cv::Vec2d censusVecL = censusLeft.at<cv::Vec2d>(cy, cx);
    cv::Vec2d censusVecR = censusRight.at<cv::Vec2d>(cy, cx);

    uint64_t censusL_low, censusR_low;
    memcpy(&censusL_low, &censusVecL[0], sizeof(uint64_t));
    memcpy(&censusR_low, &censusVecR[0], sizeof(uint64_t));

    std::cout << "Census at center (" << cx << "," << cy << "):" << std::endl;
    std::cout << "  Left:  0x" << std::hex << censusL_low << std::dec << std::endl;
    std::cout << "  Right: 0x" << std::hex << censusR_low << std::dec << std::endl;

    // Compute Hamming cost volume
    std::cout << "\n--- Computing Hamming Cost Volume ---" << std::endl;
    cv::Mat costVolume;
    const int minDisp = 0;
    const int numDisp = 256;
    computeHammingCostSimple(censusLeft, censusRight, costVolume, minDisp, numDisp);
    std::cout << "Cost volume created: " << costVolume.size[0] << "x" << costVolume.size[1]
              << "x" << costVolume.size[2] << std::endl;

    // Save cost slices at different disparities
    std::cout << "\n--- Saving Cost Volume Slices ---" << std::endl;
    saveCostSlice(costVolume, "debug_cost_d000.png", 0);
    saveCostSlice(costVolume, "debug_cost_d032.png", 32);
    saveCostSlice(costVolume, "debug_cost_d064.png", 64);
    saveCostSlice(costVolume, "debug_cost_d128.png", 128);

    // Find WTA disparity
    std::cout << "\n--- Computing WTA Disparity ---" << std::endl;
    cv::Mat disparity(costVolume.size[0], costVolume.size[1], CV_32F, cv::Scalar(0));

    for (int y = 0; y < costVolume.size[0]; y++) {
        for (int x = 0; x < costVolume.size[1]; x++) {
            const float* costPtr = costVolume.ptr<float>(y, x);

            // Find minimum cost
            float minCost = costPtr[0];
            int minDisp = 0;
            for (int d = 1; d < numDisp; d++) {
                if (costPtr[d] < minCost) {
                    minCost = costPtr[d];
                    minDisp = d;
                }
            }

            disparity.at<float>(y, x) = static_cast<float>(minDisp);
        }
    }

    // Normalize and save disparity
    double minVal, maxVal;
    cv::minMaxLoc(disparity, &minVal, &maxVal);
    std::cout << "Disparity range: " << minVal << " - " << maxVal << std::endl;

    cv::Mat disparityVis;
    disparity.convertTo(disparityVis, CV_8U, 255.0 / std::max(1.0, maxVal));
    cv::imwrite("debug_disparity_census.png", disparityVis);
    cv::imwrite("debug_disparity_census_raw.tiff", disparity);

    std::cout << "\n=== DIAGNOSTIC COMPLETE ===" << std::endl;
    std::cout << "Output files:" << std::endl;
    std::cout << "  debug_left_enhanced.png" << std::endl;
    std::cout << "  debug_right_enhanced.png" << std::endl;
    std::cout << "  debug_cost_d*.png (cost slices)" << std::endl;
    std::cout << "  debug_disparity_census.png" << std::endl;

    return 0;
}

// Simplified Census implementation for testing
void computeCensusSimple(const cv::Mat& image, cv::Mat& census, int radius) {
    const int width = image.cols;
    const int height = image.rows;

    census.create(height, width, CV_64FC2);  // Vec2d for 80-bit descriptor
    census.setTo(cv::Scalar(0, 0));

    for (int y = radius; y < height - radius; y++) {
        const uint8_t* rowPtr = image.ptr<uint8_t>(y);
        cv::Vec2d* censusPtr = census.ptr<cv::Vec2d>(y);

        for (int x = radius; x < width - radius; x++) {
            uint64_t descriptor_low = 0;
            uint16_t descriptor_high = 0;

            const uint8_t centerVal = rowPtr[x];
            int bitPos = 0;

            // 9x9 window
            for (int dy = -radius; dy <= radius; dy++) {
                const uint8_t* winRowPtr = image.ptr<uint8_t>(y + dy);
                for (int dx = -radius; dx <= radius; dx++) {
                    if (dy == 0 && dx == 0) continue;  // Skip center

                    uint8_t pixelVal = winRowPtr[x + dx];
                    bool censusbit = (pixelVal > centerVal);

                    if (bitPos < 64) {
                        descriptor_low |= (uint64_t(censusbit) << bitPos);
                    } else {
                        descriptor_high |= (uint16_t(censusbit) << (bitPos - 64));
                    }
                    bitPos++;
                }
            }

            double low_double, high_double;
            memcpy(&low_double, &descriptor_low, sizeof(double));
            memcpy(&high_double, &descriptor_high, sizeof(double));
            censusPtr[x] = cv::Vec2d(low_double, high_double);
        }
    }
}

// Simplified Hamming cost computation
void computeHammingCostSimple(const cv::Mat& censusLeft, const cv::Mat& censusRight,
                               cv::Mat& costVolume, int minDisp, int numDisp) {
    const int width = censusLeft.cols;
    const int height = censusLeft.rows;

    const int dims[3] = {height, width, numDisp};
    costVolume.create(3, dims, CV_32F);

    for (int y = 0; y < height; y++) {
        const cv::Vec2d* leftPtr = censusLeft.ptr<cv::Vec2d>(y);
        const cv::Vec2d* rightPtr = censusRight.ptr<cv::Vec2d>(y);

        for (int x = 0; x < width; x++) {
            uint64_t left_low, left_high_temp;
            memcpy(&left_low, &leftPtr[x][0], sizeof(uint64_t));
            memcpy(&left_high_temp, &leftPtr[x][1], sizeof(uint64_t));
            uint16_t left_high = static_cast<uint16_t>(left_high_temp);

            float* costPtr = costVolume.ptr<float>(y, x);

            for (int d = 0; d < numDisp; d++) {
                int xr = x - (minDisp + d);

                if (xr >= 0 && xr < width) {
                    uint64_t right_low, right_high_temp;
                    memcpy(&right_low, &rightPtr[xr][0], sizeof(uint64_t));
                    memcpy(&right_high_temp, &rightPtr[xr][1], sizeof(uint64_t));
                    uint16_t right_high = static_cast<uint16_t>(right_high_temp);

                    // Hamming distance
                    uint64_t xor_low = left_low ^ right_low;
                    uint16_t xor_high = left_high ^ right_high;
                    int hamming = __builtin_popcountll(xor_low) + __builtin_popcount(xor_high);

                    costPtr[d] = static_cast<float>(hamming);
                } else {
                    costPtr[d] = 80.0f;  // Maximum cost
                }
            }
        }
    }
}

// Save cost volume slice
void saveCostSlice(const cv::Mat& costVolume, const std::string& filename, int dispSlice) {
    const int height = costVolume.size[0];
    const int width = costVolume.size[1];

    cv::Mat slice(height, width, CV_32F);
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            const float* costPtr = costVolume.ptr<float>(y, x);
            slice.at<float>(y, x) = costPtr[dispSlice];
        }
    }

    double minVal, maxVal;
    cv::minMaxLoc(slice, &minVal, &maxVal);
    cv::Mat sliceVis;
    slice.convertTo(sliceVis, CV_8U, 255.0 / (maxVal - minVal), -minVal * 255.0 / (maxVal - minVal));
    cv::imwrite(filename, sliceVis);

    std::cout << "Saved " << filename << " (disparity " << dispSlice
              << ", cost range: " << minVal << "-" << maxVal << ")" << std::endl;
}
