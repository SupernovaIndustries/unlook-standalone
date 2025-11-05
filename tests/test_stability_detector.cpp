/**
 * @file test_stability_detector.cpp
 * @brief Unit tests for StabilityDetector
 *
 * Validates:
 * - Multi-criteria stability algorithm
 * - Gyro magnitude threshold (<0.5 deg/sec)
 * - Accel variance threshold (<0.1 m/s²)
 * - Stable duration requirement (>=500ms)
 * - Edge cases (sudden movements, sensor noise)
 * - Thread safety
 */

#include <gtest/gtest.h>
#include <unlook/hardware/StabilityDetector.hpp>
#include <unlook/hardware/BMI270Driver.hpp>
#include <unlook/core/Logger.hpp>
#include <thread>
#include <chrono>
#include <random>

using namespace unlook::hardware;

/**
 * @brief Mock BMI270Driver for testing without hardware
 */
class MockBMI270Driver : public BMI270Driver {
public:
    MockBMI270Driver() : BMI270Driver("/dev/null", 0x00) {
        // Mark as initialized without actual hardware
        initialized_ = true;
    }

    bool initialize() override {
        initialized_ = true;
        return true;
    }

    bool isInitialized() const override {
        return initialized_;
    }

    bool readIMUData(IMUData& data) override {
        std::lock_guard<std::mutex> lock(dataMutex_);
        data = mockData_;
        return data.valid;
    }

    void setMockData(const IMUData& data) {
        std::lock_guard<std::mutex> lock(dataMutex_);
        mockData_ = data;
    }

    void simulateStableState() {
        IMUData data;
        data.valid = true;
        data.gyro_x = 0.1f;  // Well below threshold
        data.gyro_y = 0.1f;
        data.gyro_z = 0.1f;
        data.accel_x = 0.0f;
        data.accel_y = 0.0f;
        data.accel_z = 9.81f;  // Gravity
        data.temperature = 25.0f;
        setMockData(data);
    }

    void simulateUnstableState() {
        IMUData data;
        data.valid = true;
        data.gyro_x = 2.0f;  // Above threshold
        data.gyro_y = 1.5f;
        data.gyro_z = 1.0f;
        data.accel_x = 1.0f;
        data.accel_y = 0.5f;
        data.accel_z = 9.81f;
        data.temperature = 25.0f;
        setMockData(data);
    }

    void simulateNoiseState() {
        std::mt19937 rng(42);
        std::normal_distribution<float> gyroDist(0.0f, 0.2f);
        std::normal_distribution<float> accelDist(0.0f, 0.05f);

        IMUData data;
        data.valid = true;
        data.gyro_x = gyroDist(rng);
        data.gyro_y = gyroDist(rng);
        data.gyro_z = gyroDist(rng);
        data.accel_x = accelDist(rng);
        data.accel_y = accelDist(rng);
        data.accel_z = 9.81f + accelDist(rng);
        data.temperature = 25.0f;
        setMockData(data);
    }

private:
    std::atomic<bool> initialized_{false};
    std::mutex dataMutex_;
    IMUData mockData_;
};

class StabilityDetectorTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Initialize logger
        unlook::core::Logger::getInstance().setLogLevel(unlook::core::LogLevel::INFO);

        // Create mock IMU driver
        mockIMU_ = std::make_shared<MockBMI270Driver>();
        mockIMU_->initialize();

        // Create stability detector
        detector_ = std::make_unique<StabilityDetector>(mockIMU_);
    }

    void TearDown() override {
        if (detector_ && detector_->isInitialized()) {
            detector_->shutdown();
        }
        detector_.reset();
        mockIMU_.reset();
    }

    std::shared_ptr<MockBMI270Driver> mockIMU_;
    std::unique_ptr<StabilityDetector> detector_;
};

/**
 * Test 1: Initialization
 */
TEST_F(StabilityDetectorTest, Initialization) {
    ASSERT_NE(detector_, nullptr);
    EXPECT_FALSE(detector_->isInitialized());

    // Initialize with default parameters
    bool success = detector_->initialize();
    EXPECT_TRUE(success);
    EXPECT_TRUE(detector_->isInitialized());

    // Check default parameters
    auto params = detector_->getParameters();
    EXPECT_FLOAT_EQ(params.gyro_threshold_dps, 0.5f);
    EXPECT_FLOAT_EQ(params.accel_variance_threshold, 0.1f);
    EXPECT_EQ(params.stable_duration_ms, 500);
    EXPECT_EQ(params.history_window_ms, 1000);

    // Weights should sum to 1.0
    EXPECT_FLOAT_EQ(params.gyro_weight + params.accel_weight, 1.0f);
}

/**
 * Test 2: Parameter Validation and Normalization
 */
TEST_F(StabilityDetectorTest, ParameterValidation) {
    StabilityDetector::StabilityParams params;
    params.gyro_threshold_dps = 0.3f;
    params.accel_variance_threshold = 0.05f;
    params.stable_duration_ms = 1000;
    params.history_window_ms = 2000;
    params.gyro_weight = 0.8f;
    params.accel_weight = 0.3f;  // Intentionally wrong (sum > 1.0)

    bool success = detector_->initialize(params);
    EXPECT_TRUE(success);

    // Weights should be normalized to sum to 1.0
    auto retrievedParams = detector_->getParameters();
    EXPECT_FLOAT_EQ(retrievedParams.gyro_weight + retrievedParams.accel_weight, 1.0f);
}

/**
 * Test 3: Stable State Detection
 */
TEST_F(StabilityDetectorTest, StableStateDetection) {
    ASSERT_TRUE(detector_->initialize());

    // Simulate stable IMU data
    mockIMU_->simulateStableState();

    // Update detector multiple times to build history
    const int numUpdates = 60;  // 600ms worth of data at 100Hz
    for (int i = 0; i < numUpdates; i++) {
        detector_->update();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    // Check stability status
    auto status = detector_->getStatus();

    EXPECT_TRUE(status.is_stable);
    EXPECT_GT(status.stability_score, 0.9f);
    EXPECT_GE(status.stable_duration_ms, 500);
    EXPECT_LT(status.current_gyro_magnitude, 0.5f);
}

/**
 * Test 4: Unstable State Detection
 */
TEST_F(StabilityDetectorTest, UnstableStateDetection) {
    ASSERT_TRUE(detector_->initialize());

    // Simulate unstable IMU data (high gyro values)
    mockIMU_->simulateUnstableState();

    // Update detector
    for (int i = 0; i < 10; i++) {
        detector_->update();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    // Check stability status
    auto status = detector_->getStatus();

    EXPECT_FALSE(status.is_stable);
    EXPECT_LT(status.stability_score, 0.5f);
    EXPECT_EQ(status.stable_duration_ms, 0);
    EXPECT_GT(status.current_gyro_magnitude, 0.5f);
}

/**
 * Test 5: Gyro Magnitude Threshold
 */
TEST_F(StabilityDetectorTest, GyroMagnitudeThreshold) {
    StabilityDetector::StabilityParams params;
    params.gyro_threshold_dps = 0.5f;
    ASSERT_TRUE(detector_->initialize(params));

    // Test just below threshold
    BMI270Driver::IMUData data;
    data.valid = true;
    data.gyro_x = 0.3f;
    data.gyro_y = 0.2f;
    data.gyro_z = 0.1f;
    data.accel_x = 0.0f;
    data.accel_y = 0.0f;
    data.accel_z = 9.81f;
    data.temperature = 25.0f;
    mockIMU_->setMockData(data);

    // Build stable history
    for (int i = 0; i < 60; i++) {
        detector_->update();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    EXPECT_TRUE(detector_->isStable());

    // Now exceed threshold
    data.gyro_x = 0.6f;  // Above threshold
    mockIMU_->setMockData(data);

    detector_->update();

    EXPECT_FALSE(detector_->isStable());
}

/**
 * Test 6: Accel Variance Threshold
 */
TEST_F(StabilityDetectorTest, AccelVarianceThreshold) {
    StabilityDetector::StabilityParams params;
    params.accel_variance_threshold = 0.1f;
    ASSERT_TRUE(detector_->initialize(params));

    // Simulate varying acceleration (high variance)
    std::mt19937 rng(42);
    std::uniform_real_distribution<float> dist(0.0f, 1.0f);

    for (int i = 0; i < 20; i++) {
        BMI270Driver::IMUData data;
        data.valid = true;
        data.gyro_x = 0.1f;
        data.gyro_y = 0.1f;
        data.gyro_z = 0.1f;
        data.accel_x = dist(rng);  // High variance
        data.accel_y = dist(rng);
        data.accel_z = 9.81f + dist(rng);
        data.temperature = 25.0f;
        mockIMU_->setMockData(data);

        detector_->update();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    auto status = detector_->getStatus();

    // High acceleration variance should prevent stability
    EXPECT_FALSE(status.is_stable);
    EXPECT_GT(status.current_accel_variance, 0.1f);
}

/**
 * Test 7: Stable Duration Requirement
 */
TEST_F(StabilityDetectorTest, StableDurationRequirement) {
    StabilityDetector::StabilityParams params;
    params.stable_duration_ms = 500;
    ASSERT_TRUE(detector_->initialize(params));

    mockIMU_->simulateStableState();

    // Update for less than required duration
    auto startTime = std::chrono::steady_clock::now();

    while (std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::steady_clock::now() - startTime).count() < 300) {
        detector_->update();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    // Should not be stable yet (< 500ms)
    EXPECT_FALSE(detector_->isStable());

    // Continue for full duration
    while (std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::steady_clock::now() - startTime).count() < 600) {
        detector_->update();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    // Now should be stable (>= 500ms)
    EXPECT_TRUE(detector_->isStable());
    EXPECT_GE(detector_->getStableDuration(), 500);
}

/**
 * Test 8: State Transitions
 */
TEST_F(StabilityDetectorTest, StateTransitions) {
    ASSERT_TRUE(detector_->initialize());

    // Start stable
    mockIMU_->simulateStableState();
    for (int i = 0; i < 60; i++) {
        detector_->update();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    EXPECT_TRUE(detector_->isStable());

    // Transition to unstable
    mockIMU_->simulateUnstableState();
    detector_->update();
    EXPECT_FALSE(detector_->isStable());
    EXPECT_EQ(detector_->getStableDuration(), 0);

    // Transition back to stable
    mockIMU_->simulateStableState();
    for (int i = 0; i < 60; i++) {
        detector_->update();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    EXPECT_TRUE(detector_->isStable());
}

/**
 * Test 9: Noise Robustness
 */
TEST_F(StabilityDetectorTest, NoiseRobustness) {
    ASSERT_TRUE(detector_->initialize());

    // Simulate noisy but stable data
    for (int i = 0; i < 60; i++) {
        mockIMU_->simulateNoiseState();
        detector_->update();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    // Should achieve stability despite noise
    auto status = detector_->getStatus();
    EXPECT_TRUE(status.is_stable);
    EXPECT_GT(status.stability_score, 0.8f);
}

/**
 * Test 10: History Window Management
 */
TEST_F(StabilityDetectorTest, HistoryWindowManagement) {
    StabilityDetector::StabilityParams params;
    params.history_window_ms = 500;  // Short window
    ASSERT_TRUE(detector_->initialize(params));

    mockIMU_->simulateStableState();

    // Add data for longer than history window
    for (int i = 0; i < 100; i++) {
        detector_->update();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    auto status = detector_->getStatus();

    // History should not exceed window size
    // At 100Hz, 500ms = 50 samples
    EXPECT_LE(status.samples_in_history, 60);  // Allow some tolerance
}

/**
 * Test 11: Stability Score Smoothness
 */
TEST_F(StabilityDetectorTest, StabilityScoreSmoothness) {
    ASSERT_TRUE(detector_->initialize());

    mockIMU_->simulateStableState();

    std::vector<float> scores;

    // Collect stability scores over time
    for (int i = 0; i < 50; i++) {
        detector_->update();
        scores.push_back(detector_->getStabilityScore());
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    // Score should increase smoothly (mostly monotonic)
    int increases = 0;
    for (size_t i = 1; i < scores.size(); i++) {
        if (scores[i] >= scores[i-1]) {
            increases++;
        }
    }

    // At least 80% should be increasing
    EXPECT_GT(increases, static_cast<int>(scores.size() * 0.8));
}

/**
 * Test 12: Reset Functionality
 */
TEST_F(StabilityDetectorTest, ResetFunctionality) {
    ASSERT_TRUE(detector_->initialize());

    // Build up stable state
    mockIMU_->simulateStableState();
    for (int i = 0; i < 60; i++) {
        detector_->update();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    EXPECT_TRUE(detector_->isStable());

    // Reset
    detector_->reset();

    // State should be cleared
    auto status = detector_->getStatus();
    EXPECT_FALSE(status.is_stable);
    EXPECT_EQ(status.stable_duration_ms, 0);
    EXPECT_EQ(status.samples_in_history, 0);
    EXPECT_FLOAT_EQ(status.stability_score, 0.0f);
}

/**
 * Test 13: Thread Safety
 */
TEST_F(StabilityDetectorTest, ThreadSafety) {
    ASSERT_TRUE(detector_->initialize());

    mockIMU_->simulateStableState();

    std::atomic<bool> stopFlag{false};
    std::atomic<int> updateCount{0};
    std::atomic<int> readCount{0};

    // Thread 1: Continuous updates
    std::thread updateThread([&]() {
        while (!stopFlag) {
            if (detector_->update()) {
                updateCount++;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    });

    // Thread 2: Continuous reads
    std::thread readThread([&]() {
        while (!stopFlag) {
            auto status = detector_->getStatus();
            if (status.stability_score >= 0.0f) {
                readCount++;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    });

    // Let threads run
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    stopFlag = true;

    updateThread.join();
    readThread.join();

    // Both threads should have executed successfully
    EXPECT_GT(updateCount.load(), 0);
    EXPECT_GT(readCount.load(), 0);
}

/**
 * Test 14: Invalid Data Handling
 */
TEST_F(StabilityDetectorTest, InvalidDataHandling) {
    ASSERT_TRUE(detector_->initialize());

    // Set invalid IMU data
    BMI270Driver::IMUData data;
    data.valid = false;
    mockIMU_->setMockData(data);

    // Update should fail gracefully
    bool success = detector_->update();
    EXPECT_FALSE(success);

    // Status should remain uninitialized
    auto status = detector_->getStatus();
    EXPECT_FALSE(status.is_stable);
}

/**
 * Test 15: Sudden Movement Detection
 */
TEST_F(StabilityDetectorTest, SuddenMovementDetection) {
    ASSERT_TRUE(detector_->initialize());

    // Build stable state
    mockIMU_->simulateStableState();
    for (int i = 0; i < 60; i++) {
        detector_->update();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    EXPECT_TRUE(detector_->isStable());

    // Sudden movement
    BMI270Driver::IMUData data;
    data.valid = true;
    data.gyro_x = 5.0f;  // Large sudden rotation
    data.gyro_y = 0.1f;
    data.gyro_z = 0.1f;
    data.accel_x = 0.0f;
    data.accel_y = 0.0f;
    data.accel_z = 9.81f;
    data.temperature = 25.0f;
    mockIMU_->setMockData(data);

    detector_->update();

    // Should immediately detect instability
    EXPECT_FALSE(detector_->isStable());
}

/**
 * Test 16: Custom Parameter Effects
 */
TEST_F(StabilityDetectorTest, CustomParameterEffects) {
    // Very strict parameters
    StabilityDetector::StabilityParams strictParams;
    strictParams.gyro_threshold_dps = 0.1f;  // Very low
    strictParams.accel_variance_threshold = 0.01f;  // Very low
    strictParams.stable_duration_ms = 1000;  // Long duration

    ASSERT_TRUE(detector_->initialize(strictParams));

    mockIMU_->simulateNoiseState();  // Normal noise

    for (int i = 0; i < 120; i++) {
        detector_->update();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    // Should be harder to achieve stability with strict parameters
    auto status = detector_->getStatus();
    // May or may not be stable depending on noise, but score should be affected
    EXPECT_LE(status.stability_score, 0.95f);
}

/**
 * Test 17: Performance - Update Latency
 */
TEST_F(StabilityDetectorTest, UpdateLatency) {
    ASSERT_TRUE(detector_->initialize());

    mockIMU_->simulateStableState();

    const int numIterations = 100;
    std::vector<double> latencies;

    for (int i = 0; i < numIterations; i++) {
        auto start = std::chrono::high_resolution_clock::now();
        detector_->update();
        auto end = std::chrono::high_resolution_clock::now();

        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
        latencies.push_back(duration.count() / 1000.0);  // Convert to ms
    }

    // Calculate statistics
    double avgLatency = std::accumulate(latencies.begin(), latencies.end(), 0.0) / latencies.size();
    double maxLatency = *std::max_element(latencies.begin(), latencies.end());

    std::cout << "Average update latency: " << avgLatency << "ms" << std::endl;
    std::cout << "Maximum update latency: " << maxLatency << "ms" << std::endl;

    // Target: < 10ms latency
    EXPECT_LT(avgLatency, 10.0);
    EXPECT_LT(maxLatency, 20.0);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
