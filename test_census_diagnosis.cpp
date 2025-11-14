/**
 * @file test_census_diagnosis.cpp
 * @brief Diagnose why Census Transform fails with VCSEL dots
 *
 * This test analyzes Census descriptors in VCSEL dot patterns to understand
 * why SGM-Census produces random matches despite visible VCSEL pattern.
 *
 * HYPOTHESIS: All VCSEL dots produce similar/identical Census descriptors,
 * causing ambiguous matching and random disparity assignments.
 */

#include <opencv2/opencv.hpp>
#include <iostream>
#include <map>
#include <vector>
#include <numeric>
#include <algorithm>

/**
 * Compute 9x9 Census descriptor for a single pixel
 * Returns 80-bit descriptor as vector of uint8_t
 */
std::vector<uint8_t> computeCensus9x9(const cv::Mat& img, int x, int y) {
    const int halfWindow = 4;  // 9x9 window
    std::vector<uint8_t> descriptor(10, 0);  // 80 bits = 10 bytes

    if (x < halfWindow || y < halfWindow ||
        x >= img.cols - halfWindow || y >= img.rows - halfWindow) {
        return descriptor;  // Border pixel - return zeros
    }

    uint8_t centerValue = img.at<uint8_t>(y, x);
    int bitIndex = 0;

    for (int dy = -halfWindow; dy <= halfWindow; dy++) {
        for (int dx = -halfWindow; dx <= halfWindow; dx++) {
            if (dx == 0 && dy == 0) continue;  // Skip center pixel

            uint8_t neighborValue = img.at<uint8_t>(y + dy, x + dx);
            bool bit = (neighborValue < centerValue);  // Census comparison

            int byteIndex = bitIndex / 8;
            int bitOffset = bitIndex % 8;
            descriptor[byteIndex] |= (bit << bitOffset);
            bitIndex++;
        }
    }

    return descriptor;
}

/**
 * Compute Hamming distance between two Census descriptors
 */
int hammingDistance(const std::vector<uint8_t>& a, const std::vector<uint8_t>& b) {
    int distance = 0;
    for (size_t i = 0; i < a.size(); i++) {
        uint8_t xorResult = a[i] ^ b[i];
        distance += __builtin_popcount(xorResult);
    }
    return distance;
}

/**
 * Convert descriptor to hex string for display
 */
std::string descriptorToHex(const std::vector<uint8_t>& desc) {
    std::string hex;
    char buf[3];
    for (uint8_t byte : desc) {
        sprintf(buf, "%02x", byte);
        hex += buf;
    }
    return hex;
}

int main() {
    std::cout << "\n=== CENSUS TRANSFORM DIAGNOSIS FOR VCSEL DOTS ===\n" << std::endl;

    // Load CLAHE-enhanced left image (where VCSEL dots are visible)
    std::string imagePath = "/home/alessandro/unlook_debug/scan_20251114_205404/01b_clahe_frame0_left.png";
    cv::Mat claheImg = cv::imread(imagePath, cv::IMREAD_GRAYSCALE);

    if (claheImg.empty()) {
        std::cerr << "ERROR: Could not load " << imagePath << std::endl;
        return 1;
    }

    std::cout << "Loaded CLAHE-enhanced image: " << claheImg.cols << "x" << claheImg.rows << std::endl;

    // Find bright pixels (potential VCSEL dots)
    std::vector<cv::Point> vcselDots;
    const int VCSEL_THRESHOLD = 200;  // Bright pixels

    for (int y = 0; y < claheImg.rows; y++) {
        for (int x = 0; x < claheImg.cols; x++) {
            if (claheImg.at<uint8_t>(y, x) > VCSEL_THRESHOLD) {
                vcselDots.push_back(cv::Point(x, y));
            }
        }
    }

    std::cout << "Found " << vcselDots.size() << " bright pixels (>200 intensity) - potential VCSEL dots" << std::endl;

    if (vcselDots.size() < 100) {
        std::cout << "\n⚠️  WARNING: Very few bright pixels detected!" << std::endl;
        std::cout << "This suggests VCSEL pattern is not visible enough in CLAHE image." << std::endl;

        // Show histogram
        cv::Mat hist;
        int histSize = 256;
        float range[] = {0, 256};
        const float* histRange = {range};
        cv::calcHist(&claheImg, 1, 0, cv::Mat(), hist, 1, &histSize, &histRange);

        std::cout << "\nIntensity histogram (top 10 bins):" << std::endl;
        std::vector<std::pair<int, float>> histPairs;
        for (int i = 0; i < 256; i++) {
            histPairs.push_back({i, hist.at<float>(i)});
        }
        std::sort(histPairs.begin(), histPairs.end(),
                  [](auto& a, auto& b) { return a.second > b.second; });

        for (int i = 0; i < 10; i++) {
            std::cout << "  Intensity " << histPairs[i].first << ": "
                      << histPairs[i].second << " pixels" << std::endl;
        }
    }

    // Sample 100 VCSEL dots randomly
    const int NUM_SAMPLES = std::min(100, (int)vcselDots.size());
    std::vector<cv::Point> sampledDots;
    std::vector<int> indices(vcselDots.size());
    std::iota(indices.begin(), indices.end(), 0);
    std::random_shuffle(indices.begin(), indices.end());

    for (int i = 0; i < NUM_SAMPLES; i++) {
        sampledDots.push_back(vcselDots[indices[i]]);
    }

    std::cout << "\nComputing Census descriptors for " << NUM_SAMPLES << " sampled VCSEL dots..." << std::endl;

    // Compute Census descriptors for all sampled dots
    std::vector<std::vector<uint8_t>> descriptors;
    for (const auto& dot : sampledDots) {
        descriptors.push_back(computeCensus9x9(claheImg, dot.x, dot.y));
    }

    // Analyze descriptor similarity
    std::cout << "\n=== DESCRIPTOR SIMILARITY ANALYSIS ===" << std::endl;

    // Count unique descriptors
    std::map<std::string, int> uniqueDescriptors;
    for (const auto& desc : descriptors) {
        uniqueDescriptors[descriptorToHex(desc)]++;
    }

    std::cout << "Unique descriptors: " << uniqueDescriptors.size() << " out of " << NUM_SAMPLES << std::endl;
    std::cout << "Uniqueness ratio: " << (100.0 * uniqueDescriptors.size() / NUM_SAMPLES) << "%" << std::endl;

    if (uniqueDescriptors.size() < NUM_SAMPLES / 2) {
        std::cout << "\n❌ CRITICAL PROBLEM: Most VCSEL dots have IDENTICAL Census descriptors!" << std::endl;
        std::cout << "   This explains why SGM-Census produces random matches." << std::endl;
        std::cout << "   Census cannot distinguish between similar-looking dots." << std::endl;
    }

    // Show most common descriptors
    std::vector<std::pair<std::string, int>> sortedDesc;
    for (const auto& pair : uniqueDescriptors) {
        sortedDesc.push_back(pair);
    }
    std::sort(sortedDesc.begin(), sortedDesc.end(),
              [](auto& a, auto& b) { return a.second > b.second; });

    std::cout << "\nMost common descriptors:" << std::endl;
    for (int i = 0; i < std::min(5, (int)sortedDesc.size()); i++) {
        std::cout << "  " << sortedDesc[i].first << ": " << sortedDesc[i].second
                  << " dots (" << (100.0 * sortedDesc[i].second / NUM_SAMPLES) << "%)" << std::endl;
    }

    // Compute pairwise Hamming distances
    std::vector<int> distances;
    for (size_t i = 0; i < descriptors.size(); i++) {
        for (size_t j = i + 1; j < descriptors.size(); j++) {
            distances.push_back(hammingDistance(descriptors[i], descriptors[j]));
        }
    }

    if (!distances.empty()) {
        std::sort(distances.begin(), distances.end());
        int minDist = distances.front();
        int maxDist = distances.back();
        int medianDist = distances[distances.size() / 2];
        double avgDist = std::accumulate(distances.begin(), distances.end(), 0.0) / distances.size();

        std::cout << "\nPairwise Hamming distances (80-bit descriptors):" << std::endl;
        std::cout << "  Min: " << minDist << " bits" << std::endl;
        std::cout << "  Median: " << medianDist << " bits" << std::endl;
        std::cout << "  Average: " << avgDist << " bits" << std::endl;
        std::cout << "  Max: " << maxDist << " bits" << std::endl;

        if (avgDist < 10) {
            std::cout << "\n❌ CRITICAL: Average Hamming distance < 10 bits!" << std::endl;
            std::cout << "   VCSEL dots are TOO SIMILAR for Census to distinguish." << std::endl;
            std::cout << "   This causes ambiguous matching and random disparity." << std::endl;
        } else if (avgDist < 20) {
            std::cout << "\n⚠️  WARNING: Average Hamming distance < 20 bits" << std::endl;
            std::cout << "   VCSEL dots may be too similar for reliable matching." << std::endl;
        } else {
            std::cout << "\n✓ Average Hamming distance seems reasonable for matching." << std::endl;
        }
    }

    std::cout << "\n=== DIAGNOSIS COMPLETE ===" << std::endl;
    std::cout << "\nRECOMMENDATIONS:" << std::endl;
    std::cout << "1. If VCSEL dots have identical descriptors → Census is WRONG algorithm" << std::endl;
    std::cout << "2. Try SAD (Sum of Absolute Differences) or NCC (Normalized Cross-Correlation)" << std::endl;
    std::cout << "3. Consider using VCSEL pattern itself (dot detection + triangulation)" << std::endl;
    std::cout << "4. Or use OpenCV's standard SGBM with texture-based matching" << std::endl;

    return 0;
}
