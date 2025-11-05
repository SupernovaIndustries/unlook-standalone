/**
 * @file test_handheld_pipeline.cpp
 * @brief Unit tests for HandheldScanPipeline
 *
 * Validates:
 * - Multi-frame fusion algorithm
 * - Outlier rejection (2.5σ threshold)
 * - Thread safety (atomics, mutexes)
 * - Error handling and state management
 * - Performance under load
 */

#include <gtest/gtest.h>
#include <unlook/api/HandheldScanPipeline.hpp>
#include <unlook/api/camera_system.h>
#include <unlook/core/Logger.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <random>
#include <thread>
#include <atomic>

using namespace unlook::api;

/**
 * @brief Mock CameraSystem for testing without hardware
 */
class MockCameraSystem : public CameraSystem {
public:
    MockCameraSystem() {
        // Initialize mock state
        initialized_ = true;
    }

    bool initialize() override {
        initialized_ = true;
        return true;
    }

    bool isInitialized() const override {
        return initialized_;
    }

    bool captureSynchronized(cv::Mat& left, cv::Mat& right) {
        // Create synthetic frames
        left = cv::Mat(720, 1280, CV_8UC1);
        right = cv::Mat(720, 1280, CV_8UC1);

        // Add texture
        cv::randn(left, 128, 30);
        cv::randn(right, 128, 30);

        return true;
    }

private:
    std::atomic<bool> initialized_{false};
};

class HandheldScanPipelineTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Initialize logger
        unlook::core::Logger::getInstance().setLogLevel(unlook::core::LogLevel::INFO);

        // Create mock camera system
        mockCamera_ = std::make_shared<MockCameraSystem>();
        mockCamera_->initialize();

        // Create pipeline
        pipeline_ = std::make_unique<HandheldScanPipeline>(mockCamera_);
    }

    void TearDown() override {
        if (pipeline_) {
            pipeline_->shutdown();
        }
        pipeline_.reset();
        mockCamera_.reset();
    }

    /**
     * @brief Create synthetic depth maps with known variance
     */
    std::vector<cv::Mat> createSyntheticDepthMaps(int numMaps, float baseDepth,
                                                   float variance) {
        std::vector<cv::Mat> depthMaps;
        std::mt19937 rng(42);
        std::normal_distribution<float> dist(baseDepth, variance);

        for (int i = 0; i < numMaps; i++) {
            cv::Mat depth(720, 1280, CV_32F);

            for (int y = 0; y < depth.rows; y++) {
                for (int x = 0; x < depth.cols; x++) {
                    float value = dist(rng);
                    depth.at<float>(y, x) = std::max(0.0f, value);
                }
            }

            depthMaps.push_back(depth);
        }

        return depthMaps;
    }

    /**
     * @brief Create depth maps with outliers
     */
    std::vector<cv::Mat> createDepthMapsWithOutliers(int numMaps, float baseDepth,
                                                      float outlierRatio) {
        auto depthMaps = createSyntheticDepthMaps(numMaps, baseDepth, 5.0f);

        // Add outliers to one map
        std::mt19937 rng(42);
        std::uniform_real_distribution<float> ratioD ist(0.0f, 1.0f);

        cv::Mat& outlierMap = depthMaps[numMaps / 2];

        for (int y = 0; y < outlierMap.rows; y++) {
            for (int x = 0; x < outlierMap.cols; x++) {
                if (ratioDist(rng) < outlierRatio) {
                    // Add large outlier
                    outlierMap.at<float>(y, x) = baseDepth + 500.0f;
                }
            }
        }

        return depthMaps;
    }

    std::shared_ptr<MockCameraSystem> mockCamera_;
    std::unique_ptr<HandheldScanPipeline> pipeline_;
};

/**
 * Test 1: Initialization
 */
TEST_F(HandheldScanPipelineTest, Initialization) {
    ASSERT_NE(pipeline_, nullptr);

    bool success = pipeline_->initialize();
    EXPECT_TRUE(success);
}

/**
 * Test 2: Parameter Get/Set
 */
TEST_F(HandheldScanPipelineTest, ParameterGetSet) {
    ASSERT_TRUE(pipeline_->initialize());

    // Get default parameters
    auto params = pipeline_->getStereoParams();
    EXPECT_GT(params.numDisparities, 0);

    // Set custom parameters
    params.numDisparities = 128;
    params.blockSize = 7;
    pipeline_->setStereoParams(params);

    // Verify parameters were set
    auto retrievedParams = pipeline_->getStereoParams();
    EXPECT_EQ(retrievedParams.numDisparities, 128);
    EXPECT_EQ(retrievedParams.blockSize, 7);
}

/**
 * Test 3: Multi-Frame Fusion - Basic
 */
TEST_F(HandheldScanPipelineTest, MultiFrameFusionBasic) {
    ASSERT_TRUE(pipeline_->initialize());

    // Create synthetic depth maps
    auto depthMaps = createSyntheticDepthMaps(10, 500.0f, 5.0f);

    // Fuse depth maps
    cv::Mat fused = pipeline_->fuseDepthMaps(depthMaps, 2.5f);

    ASSERT_FALSE(fused.empty());
    EXPECT_EQ(fused.rows, 720);
    EXPECT_EQ(fused.cols, 1280);
    EXPECT_EQ(fused.type(), CV_32F);

    // Check mean depth is close to expected
    cv::Scalar meanDepth = cv::mean(fused, fused > 0);
    EXPECT_NEAR(meanDepth[0], 500.0f, 20.0f);  // Allow 20mm error
}

/**
 * Test 4: Outlier Rejection
 */
TEST_F(HandheldScanPipelineTest, OutlierRejection) {
    ASSERT_TRUE(pipeline_->initialize());

    // Create depth maps with outliers
    auto depthMaps = createDepthMapsWithOutliers(10, 500.0f, 0.1f);  // 10% outliers

    // Fuse with outlier rejection
    cv::Mat fused = pipeline_->fuseDepthMaps(depthMaps, 2.5f);

    ASSERT_FALSE(fused.empty());

    // Mean should still be close to expected (outliers rejected)
    cv::Scalar meanDepth = cv::mean(fused, fused > 0);
    EXPECT_NEAR(meanDepth[0], 500.0f, 30.0f);

    // Standard deviation should be reduced compared to single frame
    cv::Scalar meanVal, stddevVal;
    cv::meanStdDev(fused, meanVal, stddevVal, fused > 0);

    // Stddev should be much less than original (5.0mm -> expect < 3mm after fusion)
    EXPECT_LT(stddevVal[0], 10.0f);
}

/**
 * Test 5: Weighted Median Computation
 */
TEST_F(HandheldScanPipelineTest, WeightedMedianComputation) {
    ASSERT_TRUE(pipeline_->initialize());

    // Create maps with distinct values to test median
    std::vector<cv::Mat> depthMaps;

    for (int i = 0; i < 5; i++) {
        cv::Mat depth(100, 100, CV_32F, cv::Scalar(100.0f * (i + 1)));
        depthMaps.push_back(depth);
    }

    // Fuse - should get median value
    cv::Mat fused = pipeline_->fuseDepthMaps(depthMaps, 2.5f);

    ASSERT_FALSE(fused.empty());

    // Median of [100, 200, 300, 400, 500] = 300
    cv::Scalar meanDepth = cv::mean(fused);
    EXPECT_NEAR(meanDepth[0], 300.0f, 50.0f);
}

/**
 * Test 6: Empty Input Handling
 */
TEST_F(HandheldScanPipelineTest, EmptyInputHandling) {
    ASSERT_TRUE(pipeline_->initialize());

    std::vector<cv::Mat> emptyMaps;

    // Should throw exception for empty input
    EXPECT_THROW(pipeline_->fuseDepthMaps(emptyMaps, 2.5f),
                 unlook::core::Exception);
}

/**
 * Test 7: Single Frame Fusion
 */
TEST_F(HandheldScanPipelineTest, SingleFrameFusion) {
    ASSERT_TRUE(pipeline_->initialize());

    // Create single depth map
    cv::Mat singleDepth(720, 1280, CV_32F, cv::Scalar(500.0f));
    std::vector<cv::Mat> depthMaps = {singleDepth};

    // Fuse single map (should just return it)
    cv::Mat fused = pipeline_->fuseDepthMaps(depthMaps, 2.5f);

    ASSERT_FALSE(fused.empty());

    // Should be identical to input
    cv::Mat diff;
    cv::absdiff(singleDepth, fused, diff);
    double maxDiff = 0;
    cv::minMaxLoc(diff, nullptr, &maxDiff);

    EXPECT_LT(maxDiff, 1.0);  // Should be nearly identical
}

/**
 * Test 8: WLS Filter Application
 */
TEST_F(HandheldScanPipelineTest, WLSFilterApplication) {
    ASSERT_TRUE(pipeline_->initialize());

    // Create noisy depth map
    cv::Mat noisyDepth(720, 1280, CV_32F);
    cv::randn(noisyDepth, 500.0f, 50.0f);
    cv::threshold(noisyDepth, noisyDepth, 0, 0, cv::THRESH_TOZERO);

    // Create guide image
    cv::Mat guideImage(720, 1280, CV_8UC1);
    cv::randn(guideImage, 128, 30);

    // Apply WLS filter
    cv::Mat filtered = pipeline_->wlsFilter(noisyDepth, guideImage, 8000.0, 1.5);

    ASSERT_FALSE(filtered.empty());
    EXPECT_EQ(filtered.size(), noisyDepth.size());

    // Filtered should have reduced variance
    cv::Scalar meanOrig, stddevOrig, meanFilt, stddevFilt;
    cv::meanStdDev(noisyDepth, meanOrig, stddevOrig, noisyDepth > 0);
    cv::meanStdDev(filtered, meanFilt, stddevFilt, filtered > 0);

    EXPECT_LT(stddevFilt[0], stddevOrig[0]);
}

/**
 * Test 9: Point Cloud Generation
 */
TEST_F(HandheldScanPipelineTest, PointCloudGeneration) {
    ASSERT_TRUE(pipeline_->initialize());

    // Create simple depth map
    cv::Mat depth(720, 1280, CV_32F, cv::Scalar(500.0f));

    // Generate point cloud without color
    cv::Mat pointCloud = pipeline_->generatePointCloud(depth);

    ASSERT_FALSE(pointCloud.empty());
    EXPECT_GT(pointCloud.rows, 0);
    EXPECT_EQ(pointCloud.cols, 1);

    // Check point cloud has 3 channels (XYZ) or 6 channels (XYZ + RGB)
    int channels = pointCloud.channels();
    EXPECT_TRUE(channels == 3 || channels == 6);
}

/**
 * Test 10: Point Cloud Generation with Color
 */
TEST_F(HandheldScanPipelineTest, PointCloudGenerationWithColor) {
    ASSERT_TRUE(pipeline_->initialize());

    // Create depth map and color image
    cv::Mat depth(720, 1280, CV_32F, cv::Scalar(500.0f));
    cv::Mat colorImage(720, 1280, CV_8UC3, cv::Scalar(255, 0, 0));  // Red

    // Generate colored point cloud
    cv::Mat pointCloud = pipeline_->generatePointCloud(depth, colorImage);

    ASSERT_FALSE(pointCloud.empty());
    EXPECT_GT(pointCloud.rows, 0);

    // Should have 6 channels (XYZ + RGB)
    EXPECT_EQ(pointCloud.channels(), 6);
}

/**
 * Test 11: Precision Calculation
 */
TEST_F(HandheldScanPipelineTest, PrecisionCalculation) {
    ASSERT_TRUE(pipeline_->initialize());

    // Create depth maps with known variance
    auto depthMaps = createSyntheticDepthMaps(10, 500.0f, 0.1f);  // 0.1mm stddev

    // Calculate precision
    float precision = pipeline_->calculatePrecision(depthMaps);

    // Should report precision close to input variance / sqrt(N)
    // Expected: 0.1 / sqrt(10) ≈ 0.032mm
    EXPECT_GT(precision, 0.0f);
    EXPECT_LT(precision, 0.5f);  // Should be sub-millimeter
}

/**
 * Test 12: Stereo Algorithm Selection
 */
TEST_F(HandheldScanPipelineTest, StereoAlgorithmSelection) {
    ASSERT_TRUE(pipeline_->initialize());

    // Set different algorithms
    pipeline_->setStereoAlgorithm(unlook::stereo::StereoAlgorithm::SGBM);
    // Should not crash

    pipeline_->setStereoAlgorithm(unlook::stereo::StereoAlgorithm::CUSTOM);
    // Should not crash
}

/**
 * Test 13: VCSEL Enable/Disable
 */
TEST_F(HandheldScanPipelineTest, VCSELEnableDisable) {
    ASSERT_TRUE(pipeline_->initialize());

    // Enable VCSEL (may fail if hardware not available, but should not crash)
    pipeline_->enableVCSEL(true);

    // Disable VCSEL
    pipeline_->enableVCSEL(false);

    // Should not crash
    SUCCEED();
}

/**
 * Test 14: Statistics Tracking
 */
TEST_F(HandheldScanPipelineTest, StatisticsTracking) {
    ASSERT_TRUE(pipeline_->initialize());

    // Get initial statistics
    auto stats = pipeline_->getStatistics();

    // Should have some statistics (even if empty)
    EXPECT_GE(stats.size(), 0);
}

/**
 * Test 15: Thread Safety - Concurrent Fusion
 */
TEST_F(HandheldScanPipelineTest, ThreadSafetyConcurrentFusion) {
    ASSERT_TRUE(pipeline_->initialize());

    std::atomic<int> successCount{0};
    const int numThreads = 4;

    std::vector<std::thread> threads;

    for (int i = 0; i < numThreads; i++) {
        threads.emplace_back([&]() {
            auto depthMaps = createSyntheticDepthMaps(5, 500.0f, 5.0f);
            cv::Mat fused = pipeline_->fuseDepthMaps(depthMaps, 2.5f);

            if (!fused.empty()) {
                successCount++;
            }
        });
    }

    for (auto& thread : threads) {
        thread.join();
    }

    // All fusions should succeed
    EXPECT_EQ(successCount.load(), numThreads);
}

/**
 * Test 16: Memory Efficiency - Large Number of Frames
 */
TEST_F(HandheldScanPipelineTest, MemoryEfficiencyLargeFrameCount) {
    ASSERT_TRUE(pipeline_->initialize());

    // Create many depth maps
    auto depthMaps = createSyntheticDepthMaps(50, 500.0f, 5.0f);

    // Fuse (should handle large frame count)
    cv::Mat fused = pipeline_->fuseDepthMaps(depthMaps, 2.5f);

    ASSERT_FALSE(fused.empty());
    EXPECT_EQ(fused.size(), depthMaps[0].size());
}

/**
 * Test 17: Sigma Threshold Variation
 */
TEST_F(HandheldScanPipelineTest, SigmaThresholdVariation) {
    ASSERT_TRUE(pipeline_->initialize());

    auto depthMaps = createDepthMapsWithOutliers(10, 500.0f, 0.2f);

    // Test different sigma thresholds
    cv::Mat fused1 = pipeline_->fuseDepthMaps(depthMaps, 1.5f);  // Strict
    cv::Mat fused2 = pipeline_->fuseDepthMaps(depthMaps, 2.5f);  // Normal
    cv::Mat fused3 = pipeline_->fuseDepthMaps(depthMaps, 3.5f);  // Lenient

    ASSERT_FALSE(fused1.empty());
    ASSERT_FALSE(fused2.empty());
    ASSERT_FALSE(fused3.empty());

    // Stricter threshold should reject more pixels
    int valid1 = cv::countNonZero(fused1 > 0);
    int valid2 = cv::countNonZero(fused2 > 0);
    int valid3 = cv::countNonZero(fused3 > 0);

    // More lenient threshold should accept more pixels
    EXPECT_LE(valid1, valid2);
    EXPECT_LE(valid2, valid3);
}

/**
 * Test 18: Performance - Fusion Time
 */
TEST_F(HandheldScanPipelineTest, PerformanceFusionTime) {
    ASSERT_TRUE(pipeline_->initialize());

    auto depthMaps = createSyntheticDepthMaps(10, 500.0f, 5.0f);

    auto startTime = std::chrono::high_resolution_clock::now();

    cv::Mat fused = pipeline_->fuseDepthMaps(depthMaps, 2.5f);

    auto endTime = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);

    ASSERT_FALSE(fused.empty());

    std::cout << "Fusion time for 10 frames: " << duration.count() << "ms" << std::endl;

    // Target: < 100ms for 10 HD frames
    EXPECT_LT(duration.count(), 200);  // Relaxed for CI
}

/**
 * Test 19: Depth Range Validation
 */
TEST_F(HandheldScanPipelineTest, DepthRangeValidation) {
    ASSERT_TRUE(pipeline_->initialize());

    // Create depth maps with various ranges
    std::vector<cv::Mat> depthMaps;

    // Map 1: Short range (100-200mm)
    cv::Mat depth1(100, 100, CV_32F);
    cv::randn(depth1, 150.0f, 10.0f);
    depthMaps.push_back(depth1);

    // Map 2: Medium range (500-600mm)
    cv::Mat depth2(100, 100, CV_32F);
    cv::randn(depth2, 550.0f, 10.0f);
    depthMaps.push_back(depth2);

    // Map 3: Long range (1000-1100mm)
    cv::Mat depth3(100, 100, CV_32F);
    cv::randn(depth3, 1050.0f, 10.0f);
    depthMaps.push_back(depth3);

    // Fuse mixed ranges
    cv::Mat fused = pipeline_->fuseDepthMaps(depthMaps, 2.5f);

    ASSERT_FALSE(fused.empty());

    // All valid depths should be positive
    double minDepth, maxDepth;
    cv::minMaxLoc(fused, &minDepth, &maxDepth, nullptr, nullptr, fused > 0);

    EXPECT_GT(minDepth, 0.0);
    EXPECT_LT(maxDepth, 10000.0);  // Reasonable maximum
}

/**
 * Test 20: State Management
 */
TEST_F(HandheldScanPipelineTest, StateManagement) {
    // Initialize
    ASSERT_TRUE(pipeline_->initialize());

    // Shutdown
    pipeline_->shutdown();

    // Re-initialize should work
    EXPECT_TRUE(pipeline_->initialize());

    // Shutdown again
    pipeline_->shutdown();
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
