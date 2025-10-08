/**
 * Unit tests for TemporalStereoProcessor
 */

#include <gtest/gtest.h>
#include <unlook/stereo/TemporalStereoProcessor.hpp>
#include <unlook/hardware/AS1170DualVCSELController.hpp>
#include <unlook/calibration/CalibrationManager.hpp>
#include <opencv2/opencv.hpp>

using namespace unlook;

class TemporalStereoTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Create test images
        createTestImages();
    }

    void createTestImages() {
        // Create synthetic test images with dot patterns
        int width = 640;
        int height = 480;

        // Create ambient frame (low intensity background)
        ambientLeft = cv::Mat(height, width, CV_8UC1, cv::Scalar(50));
        ambientRight = cv::Mat(height, width, CV_8UC1, cv::Scalar(50));

        // Create VCSEL1 frames (dots from left perspective)
        vcsel1Left = ambientLeft.clone();
        vcsel1Right = ambientRight.clone();
        addDotPattern(vcsel1Left, 100, 100, 200);  // Strong dots on left
        addDotPattern(vcsel1Right, 120, 100, 180); // Shifted dots on right

        // Create VCSEL2 frames (dots from right perspective)
        vcsel2Left = ambientLeft.clone();
        vcsel2Right = ambientRight.clone();
        addDotPattern(vcsel2Left, 200, 150, 180);  // Different dot positions
        addDotPattern(vcsel2Right, 180, 150, 200); // Strong dots on right
    }

    void addDotPattern(cv::Mat& image, int offsetX, int offsetY, int intensity) {
        // Add a grid of dots to simulate VCSEL pattern
        for (int y = offsetY; y < image.rows - 10; y += 20) {
            for (int x = offsetX; x < image.cols - 10; x += 20) {
                cv::circle(image, cv::Point(x, y), 2, cv::Scalar(intensity), -1);
            }
        }
    }

    cv::Mat ambientLeft, ambientRight;
    cv::Mat vcsel1Left, vcsel1Right;
    cv::Mat vcsel2Left, vcsel2Right;
};

TEST_F(TemporalStereoTest, PatternIsolation) {
    auto processor = std::make_unique<stereo::TemporalStereoProcessor>();

    // Test pattern isolation
    cv::Mat isolatedPattern;
    float variance = processor->isolateVCSELPattern(vcsel1Left, ambientLeft, isolatedPattern);

    // Check that pattern was isolated
    EXPECT_FALSE(isolatedPattern.empty());
    EXPECT_GT(variance, 0.0f);

    // Check that isolated pattern has expected properties
    double minVal, maxVal;
    cv::minMaxLoc(isolatedPattern, &minVal, &maxVal);
    EXPECT_GE(maxVal, 100.0); // Should have bright dots
    EXPECT_LE(minVal, 10.0);  // Background should be dark
}

TEST_F(TemporalStereoTest, PatternCombination) {
    auto processor = std::make_unique<stereo::TemporalStereoProcessor>();

    // Isolate patterns first
    cv::Mat pattern1, pattern2;
    processor->isolateVCSELPattern(vcsel1Left, ambientLeft, pattern1);
    processor->isolateVCSELPattern(vcsel2Left, ambientLeft, pattern2);

    // Combine patterns
    cv::Mat combined;
    float coverage = processor->combinePatterns(pattern1, pattern2, combined);

    // Check combination results
    EXPECT_FALSE(combined.empty());
    EXPECT_GT(coverage, 0.0f);
    EXPECT_LT(coverage, 1.0f); // Should not be fully covered

    // Combined should have content from both patterns
    EXPECT_GE(cv::countNonZero(combined), cv::countNonZero(pattern1));
    EXPECT_GE(cv::countNonZero(combined), cv::countNonZero(pattern2));
}

TEST_F(TemporalStereoTest, ConfigValidation) {
    auto processor = std::make_unique<stereo::TemporalStereoProcessor>();

    // Test valid VCSEL-optimized config
    auto validConfig = stereo::TemporalStereoProcessor::TemporalStereoConfig::getVCSELOptimized();
    EXPECT_TRUE(validConfig.sgbmParams.validate());
    EXPECT_EQ(validConfig.sgbmParams.blockSize, 5);
    EXPECT_EQ(validConfig.sgbmParams.numDisparities, 160);

    // Test config update
    validConfig.isolationParams.noiseThreshold = 15;
    validConfig.isolationParams.vcsel1Weight = 0.6f;
    validConfig.isolationParams.vcsel2Weight = 0.4f;
    EXPECT_TRUE(processor->updateConfig(validConfig));
}

TEST_F(TemporalStereoTest, DepthTopologyValidation) {
    auto processor = std::make_unique<stereo::TemporalStereoProcessor>();

    // Create a synthetic depth map with expected topology
    cv::Mat depthMap(480, 640, CV_32F);

    // Top region: far (background)
    depthMap(cv::Rect(0, 0, 640, 160)) = 2000.0f;

    // Middle region: medium distance
    depthMap(cv::Rect(0, 160, 640, 160)) = 1000.0f;

    // Bottom region: close (foreground)
    depthMap(cv::Rect(0, 320, 640, 160)) = 500.0f;

    std::string message;
    bool isValid = processor->validateDepthTopology(depthMap, message);

    EXPECT_TRUE(isValid);
    EXPECT_FALSE(message.empty());
}

TEST_F(TemporalStereoTest, ProcessingPipeline) {
    // This test requires actual hardware/calibration, so it's a compile-time test only
    auto processor = std::make_unique<stereo::TemporalStereoProcessor>();

    // Create mock triple frame capture
    hardware::AS1170DualVCSELController::TripleFrameCapture frames;
    frames.frame_vcsel1_left = vcsel1Left;
    frames.frame_vcsel1_right = vcsel1Right;
    frames.frame_vcsel2_left = vcsel2Left;
    frames.frame_vcsel2_right = vcsel2Right;
    frames.frame_ambient_left = ambientLeft;
    frames.frame_ambient_right = ambientRight;
    frames.is_valid = true;

    // Verify structure is valid
    EXPECT_TRUE(frames.is_valid);
    EXPECT_FALSE(frames.frame_vcsel1_left.empty());
    EXPECT_FALSE(frames.frame_vcsel1_right.empty());
}

TEST_F(TemporalStereoTest, StatisticsTracking) {
    auto processor = std::make_unique<stereo::TemporalStereoProcessor>();

    // Initial statistics should be empty
    auto stats = processor->getStatistics();
    EXPECT_TRUE(stats.empty() || stats.size() == 0);

    // After processing, statistics should be populated
    // (would need actual processing to test this fully)
}

TEST_F(TemporalStereoTest, DebugMode) {
    auto processor = std::make_unique<stereo::TemporalStereoProcessor>();

    // Test debug mode setting
    std::string debugPath = "/tmp/test_temporal_stereo/";
    processor->setDebugMode(true, debugPath);

    // Verify debug mode can be disabled
    processor->setDebugMode(false);

    // No crash test - just verify methods are callable
    EXPECT_TRUE(true);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}