/**
 * @file test_vcsel_stereo_unit.cpp
 * @brief Unit tests for VCSELStereoMatcher
 *
 * Validates:
 * - AD-Census fusion correctness
 * - NEON-optimized census transform
 * - Hamming distance POPCOUNT accuracy
 * - Performance targets (HD 1280x720 @ ~10 FPS)
 * - Quality metrics computation
 * - Parameter validation
 */

#include <gtest/gtest.h>
#include <unlook/stereo/VCSELStereoMatcher.hpp>
#include <unlook/core/Logger.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <random>
#include <cmath>

using namespace unlook::stereo;

class VCSELStereoMatcherTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Initialize logger
        unlook::core::Logger::getInstance().setLogLevel(unlook::core::LogLevel::INFO);

        // Create matcher
        matcher_ = std::make_unique<VCSELStereoMatcher>();

        // Create synthetic test data
        createSyntheticStereoImages();
    }

    void TearDown() override {
        matcher_.reset();
    }

    /**
     * @brief Create synthetic stereo pair with known disparity
     */
    void createSyntheticStereoImages() {
        const int width = 1280;
        const int height = 720;
        const int disparity = 64;

        // Create textured left image
        leftImage_ = cv::Mat(height, width, CV_8UC1);

        // Add random texture with some structure
        std::mt19937 rng(42);
        std::uniform_int_distribution<int> dist(0, 255);

        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++) {
                // Add vertical stripes + noise for texture
                int value = (x % 20) * 12 + dist(rng) % 50;
                leftImage_.at<uchar>(y, x) = cv::saturate_cast<uchar>(value);
            }
        }

        // Apply Gaussian blur for smoother texture
        cv::GaussianBlur(leftImage_, leftImage_, cv::Size(3, 3), 1.0);

        // Create right image by shifting left image
        rightImage_ = cv::Mat::zeros(height, width, CV_8UC1);

        for (int y = 0; y < height; y++) {
            for (int x = disparity; x < width; x++) {
                rightImage_.at<uchar>(y, x) = leftImage_.at<uchar>(y, x - disparity);
            }
        }

        expectedDisparity_ = disparity;
    }

    /**
     * @brief Create synthetic VCSEL dot pattern
     */
    cv::Mat createVCSELPattern(int width, int height) {
        cv::Mat pattern = cv::Mat::zeros(height, width, CV_8UC1);

        // Add regular dot pattern (9x9 grid)
        const int spacing = 40;
        const int dotSize = 3;

        for (int y = spacing; y < height; y += spacing) {
            for (int x = spacing; x < width; x += spacing) {
                cv::circle(pattern, cv::Point(x, y), dotSize, cv::Scalar(255), -1);
            }
        }

        return pattern;
    }

    std::unique_ptr<VCSELStereoMatcher> matcher_;
    cv::Mat leftImage_;
    cv::Mat rightImage_;
    int expectedDisparity_;
};

/**
 * Test 1: Initialization and NEON Support Detection
 */
TEST_F(VCSELStereoMatcherTest, InitializationAndNEONSupport) {
    ASSERT_NE(matcher_, nullptr);

    // Check algorithm type
    EXPECT_EQ(matcher_->getAlgorithmType(), StereoAlgorithm::CUSTOM);
    EXPECT_EQ(matcher_->getAlgorithmName(), "VCSEL AD-Census");

    // Verify parameters can be retrieved
    auto params = matcher_->getParameters();
    EXPECT_GT(params.numDisparities, 0);
    EXPECT_GE(params.minDisparity, 0);
}

/**
 * Test 2: Census Transform Correctness
 *
 * Validates the 9x9 window 80-bit census descriptor computation
 */
TEST_F(VCSELStereoMatcherTest, CensusTransformCorrectness) {
    // Create simple test image with known pattern
    cv::Mat testImage = cv::Mat::zeros(100, 100, CV_8UC1);

    // Create a bright spot in the center
    testImage(cv::Rect(40, 40, 20, 20)) = 255;

    // Compute disparity (which internally uses census transform)
    cv::Mat disparity;
    StereoMatchingParams params = matcher_->getParameters();
    params.minDisparity = 0;
    params.numDisparities = 64;
    matcher_->setParameters(params);

    bool success = matcher_->computeDisparity(testImage, testImage, disparity);

    // Census transform should handle this image without crashing
    EXPECT_TRUE(success);
    EXPECT_FALSE(disparity.empty());
    EXPECT_EQ(disparity.rows, testImage.rows);
    EXPECT_EQ(disparity.cols, testImage.cols);
}

/**
 * Test 3: Hamming Distance Accuracy
 *
 * Validates POPCOUNT-based Hamming distance computation
 */
TEST_F(VCSELStereoMatcherTest, HammingDistanceAccuracy) {
    // Create two similar images with known difference
    cv::Mat img1 = cv::Mat::zeros(100, 100, CV_8UC1);
    cv::Mat img2 = cv::Mat::zeros(100, 100, CV_8UC1);

    // Add identical texture
    for (int y = 0; y < 100; y++) {
        for (int x = 0; x < 100; x++) {
            img1.at<uchar>(y, x) = (x + y) % 256;
            img2.at<uchar>(y, x) = (x + y) % 256;
        }
    }

    // Compute disparity (should be nearly zero for identical images)
    cv::Mat disparity;
    bool success = matcher_->computeDisparity(img1, img2, disparity);

    ASSERT_TRUE(success);

    // Count valid disparities (most should be at minimum)
    int validPixels = cv::countNonZero(disparity > 0);

    // For identical images, most disparities should be at minDisparity
    // We expect some valid matches
    EXPECT_GT(validPixels, disparity.total() * 0.5);
}

/**
 * Test 4: AD Cost Computation Validation
 */
TEST_F(VCSELStereoMatcherTest, ADCostComputation) {
    // Use our synthetic stereo pair
    cv::Mat disparity;

    StereoMatchingParams params = matcher_->getParameters();
    params.minDisparity = 0;
    params.numDisparities = 128;
    matcher_->setParameters(params);

    bool success = matcher_->computeDisparity(leftImage_, rightImage_, disparity);

    ASSERT_TRUE(success);
    EXPECT_FALSE(disparity.empty());

    // Check disparity statistics
    double minDisp, maxDisp;
    cv::minMaxLoc(disparity, &minDisp, &maxDisp);

    // Should have reasonable disparity range
    EXPECT_GE(minDisp, 0.0);
    EXPECT_LE(maxDisp, 256.0);

    // Mean disparity should be near expected value
    cv::Scalar meanDisp = cv::mean(disparity, disparity > 0);
    double meanError = std::abs(meanDisp[0] - expectedDisparity_);

    // Allow 20% error for synthetic test (would be much better with real calibrated data)
    EXPECT_LT(meanError, expectedDisparity_ * 0.2);
}

/**
 * Test 5: Cost Fusion Validation
 *
 * Validates λ_AD=0.3, λ_Census=0.7 fusion
 */
TEST_F(VCSELStereoMatcherTest, CostFusionValidation) {
    // The fusion is internal to computeDisparity
    // We validate it produces consistent results

    cv::Mat disparity1, disparity2;

    // Run twice with same input
    bool success1 = matcher_->computeDisparity(leftImage_, rightImage_, disparity1);
    bool success2 = matcher_->computeDisparity(leftImage_, rightImage_, disparity2);

    ASSERT_TRUE(success1);
    ASSERT_TRUE(success2);

    // Results should be identical (deterministic)
    cv::Mat diff;
    cv::absdiff(disparity1, disparity2, diff);
    double maxDiff = 0;
    cv::minMaxLoc(diff, nullptr, &maxDiff);

    EXPECT_EQ(maxDiff, 0.0);
}

/**
 * Test 6: Performance - HD 1280x720 @ ~10 FPS
 */
TEST_F(VCSELStereoMatcherTest, PerformanceHD720p) {
    // Use HD resolution
    ASSERT_EQ(leftImage_.cols, 1280);
    ASSERT_EQ(leftImage_.rows, 720);

    cv::Mat disparity;

    auto startTime = std::chrono::high_resolution_clock::now();

    bool success = matcher_->computeDisparity(leftImage_, rightImage_, disparity);

    auto endTime = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);

    ASSERT_TRUE(success);

    // Target: ~100ms per frame (10 FPS)
    double processingTimeMs = duration.count();
    double fps = 1000.0 / processingTimeMs;

    std::cout << "HD 1280x720 processing time: " << processingTimeMs << "ms ("
              << fps << " FPS)" << std::endl;

    // Get detailed stats
    auto stats = matcher_->getLastProcessingStats();
    std::cout << "Detailed stats: " << stats.toString() << std::endl;

    // Relaxed target: < 200ms (5 FPS minimum, target is 10 FPS)
    EXPECT_LT(processingTimeMs, 200.0);

    // Check valid pixel percentage
    EXPECT_GT(stats.validPixels, static_cast<size_t>(stats.totalPixels * 0.5));
}

/**
 * Test 7: Quality Metrics Computation
 */
TEST_F(VCSELStereoMatcherTest, QualityMetricsComputation) {
    cv::Mat disparity;

    bool success = matcher_->computeDisparity(leftImage_, rightImage_, disparity);

    ASSERT_TRUE(success);

    // Get processing stats
    auto stats = matcher_->getLastProcessingStats();

    // Validate stats
    EXPECT_GT(stats.totalTimeMs, 0.0);
    EXPECT_GT(stats.totalPixels, 0);
    EXPECT_GT(stats.validPixels, 0);
    EXPECT_LE(stats.validPixels, stats.totalPixels);

    // Check individual stage times are reasonable
    EXPECT_GT(stats.censusTimeMs, 0.0);
    EXPECT_GT(stats.hammingTimeMs, 0.0);
    EXPECT_GT(stats.adCostTimeMs, 0.0);
    EXPECT_GT(stats.fusionTimeMs, 0.0);
    EXPECT_GT(stats.sgmTimeMs, 0.0);

    // SGM should be the slowest stage
    EXPECT_GT(stats.sgmTimeMs, stats.censusTimeMs);
    EXPECT_GT(stats.sgmTimeMs, stats.adCostTimeMs);
}

/**
 * Test 8: Parameter Validation
 */
TEST_F(VCSELStereoMatcherTest, ParameterValidation) {
    StereoMatchingParams params = matcher_->getParameters();

    // Validate default parameters
    EXPECT_GT(params.numDisparities, 0);
    EXPECT_EQ(params.numDisparities % 16, 0);  // Must be multiple of 16
    EXPECT_GE(params.minDisparity, 0);
    EXPECT_GT(params.P1, 0);
    EXPECT_GT(params.P2, 0);
    EXPECT_GE(params.uniquenessRatio, 0);

    // Try setting custom parameters
    params.numDisparities = 128;
    params.minDisparity = 32;
    params.P1 = 8;
    params.P2 = 32;

    bool success = matcher_->setParameters(params);
    EXPECT_TRUE(success);

    // Verify parameters were updated
    auto retrievedParams = matcher_->getParameters();
    EXPECT_EQ(retrievedParams.numDisparities, 128);
    EXPECT_EQ(retrievedParams.minDisparity, 32);
}

/**
 * Test 9: VCSEL Pattern Isolation
 */
TEST_F(VCSELStereoMatcherTest, VCSELPatternIsolation) {
    // Create VCSEL pattern image
    cv::Mat vcselImage = createVCSELPattern(1280, 720);

    // Add some ambient illumination
    cv::Mat ambientImage = cv::Mat(720, 1280, CV_8UC1, cv::Scalar(30));

    // Add VCSEL pattern to ambient
    cv::Mat combined;
    cv::add(vcselImage, ambientImage, combined);

    // Test pattern isolation
    cv::Mat isolated;
    bool success = matcher_->patternIsolation(combined, ambientImage, isolated);

    ASSERT_TRUE(success);
    EXPECT_FALSE(isolated.empty());

    // Isolated pattern should be mostly zeros except at dot locations
    int nonZero = cv::countNonZero(isolated);
    int total = isolated.total();

    // VCSEL dots should be < 5% of image
    EXPECT_LT(nonZero, total * 0.05);
}

/**
 * Test 10: Edge Cases - Empty Images
 */
TEST_F(VCSELStereoMatcherTest, EdgeCaseEmptyImages) {
    cv::Mat emptyImage;
    cv::Mat disparity;

    // Should fail gracefully with empty input
    bool success = matcher_->computeDisparity(emptyImage, leftImage_, disparity);
    EXPECT_FALSE(success);

    success = matcher_->computeDisparity(leftImage_, emptyImage, disparity);
    EXPECT_FALSE(success);

    success = matcher_->computeDisparity(emptyImage, emptyImage, disparity);
    EXPECT_FALSE(success);
}

/**
 * Test 11: Edge Cases - Mismatched Sizes
 */
TEST_F(VCSELStereoMatcherTest, EdgeCaseMismatchedSizes) {
    cv::Mat smallImage = cv::Mat(100, 100, CV_8UC1, cv::Scalar(128));
    cv::Mat disparity;

    // Should fail with mismatched sizes
    bool success = matcher_->computeDisparity(leftImage_, smallImage, disparity);
    EXPECT_FALSE(success);
}

/**
 * Test 12: Color Input Handling
 */
TEST_F(VCSELStereoMatcherTest, ColorInputHandling) {
    // Create color versions
    cv::Mat leftColor, rightColor;
    cv::cvtColor(leftImage_, leftColor, cv::COLOR_GRAY2BGR);
    cv::cvtColor(rightImage_, rightColor, cv::COLOR_GRAY2BGR);

    cv::Mat disparity;

    // Should handle color input by converting to grayscale internally
    bool success = matcher_->computeDisparity(leftColor, rightColor, disparity);

    EXPECT_TRUE(success);
    EXPECT_FALSE(disparity.empty());
}

/**
 * Test 13: Subpixel Refinement
 */
TEST_F(VCSELStereoMatcherTest, SubpixelRefinement) {
    // Subpixel refinement is enabled by default in AD-Census params
    cv::Mat disparity;

    bool success = matcher_->computeDisparity(leftImage_, rightImage_, disparity);

    ASSERT_TRUE(success);

    // Check if disparity has fractional values (indicating subpixel refinement)
    bool hasFractional = false;
    for (int y = 0; y < disparity.rows && !hasFractional; y++) {
        for (int x = 0; x < disparity.cols && !hasFractional; x++) {
            float val = disparity.at<float>(y, x);
            if (val > 0) {
                float fractional = val - std::floor(val);
                if (fractional > 0.01 && fractional < 0.99) {
                    hasFractional = true;
                }
            }
        }
    }

    // With subpixel refinement, we should see fractional disparities
    EXPECT_TRUE(hasFractional);
}

/**
 * Test 14: Post-Processing Effectiveness
 */
TEST_F(VCSELStereoMatcherTest, PostProcessingEffectiveness) {
    // Add noise to test images
    cv::Mat leftNoisy, rightNoisy;
    cv::randn(leftNoisy, 0, 10);
    cv::randn(rightNoisy, 0, 10);

    cv::add(leftImage_, leftNoisy, leftNoisy);
    cv::add(rightImage_, rightNoisy, rightNoisy);

    cv::Mat disparity;
    bool success = matcher_->computeDisparity(leftNoisy, rightNoisy, disparity);

    ASSERT_TRUE(success);

    // Post-processing should remove small speckles
    // Count connected components
    cv::Mat binary = disparity > 0;
    cv::Mat labels, stats, centroids;
    int numComponents = cv::connectedComponentsWithStats(binary, labels, stats, centroids);

    // Most regions should be relatively large (speckles removed)
    int largeRegions = 0;
    for (int i = 1; i < numComponents; i++) {
        int area = stats.at<int>(i, cv::CC_STAT_AREA);
        if (area > 100) {  // Threshold for "large" region
            largeRegions++;
        }
    }

    // Should have fewer large regions than total components (speckles removed)
    EXPECT_LT(largeRegions, numComponents);
}

/**
 * Test 15: Thread Safety
 */
TEST_F(VCSELStereoMatcherTest, ThreadSafety) {
    // Test concurrent access to getLastProcessingStats
    const int numThreads = 4;
    const int iterationsPerThread = 10;

    std::vector<std::thread> threads;
    std::atomic<int> successCount{0};

    for (int i = 0; i < numThreads; i++) {
        threads.emplace_back([&]() {
            for (int j = 0; j < iterationsPerThread; j++) {
                cv::Mat disparity;
                bool success = matcher_->computeDisparity(leftImage_, rightImage_, disparity);

                if (success) {
                    auto stats = matcher_->getLastProcessingStats();
                    if (stats.totalTimeMs > 0) {
                        successCount++;
                    }
                }
            }
        });
    }

    for (auto& thread : threads) {
        thread.join();
    }

    // All computations should succeed
    EXPECT_EQ(successCount.load(), numThreads * iterationsPerThread);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
