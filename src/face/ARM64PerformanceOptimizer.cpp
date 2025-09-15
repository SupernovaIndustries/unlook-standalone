#include "unlook/face/FaceTypes.hpp"
#include "unlook/core/logging.hpp"
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <numeric>
#include <chrono>
#include <cmath>
#include <thread>
#include <future>

#ifdef __ARM_NEON
#include <arm_neon.h>
#endif

namespace unlook {
namespace face {

/**
 * @brief ARM64/CM4/CM5 performance optimizer for facial recognition pipeline
 *
 * This class provides NEON SIMD optimizations and multi-threading
 * strategies specifically designed for ARM64 processors to achieve
 * <200ms total processing time for banking-grade facial recognition.
 */
class ARM64PerformanceOptimizer {
public:
    /**
     * @brief Performance optimization configuration
     */
    struct OptimizationConfig {
        // Threading configuration
        int max_threads = 4;                    // Maximum threads for CM4/CM5
        bool enable_parallel_processing = true; // Enable parallel algorithms
        bool enable_cpu_affinity = true;        // Set CPU affinity for threads

        // NEON SIMD optimizations
        bool enable_neon_optimizations = true;  // Enable NEON SIMD instructions
        bool enable_vectorized_matching = true; // Vectorized template matching
        bool enable_fast_math = true;           // Enable fast math optimizations

        // Memory optimizations
        bool enable_memory_pooling = true;      // Use memory pools
        bool enable_cache_optimization = true;  // Optimize cache usage
        size_t memory_pool_size_mb = 64;        // Memory pool size

        // Algorithm optimizations
        bool enable_early_termination = true;   // Early termination in algorithms
        bool enable_quality_gating = true;      // Skip processing for low quality
        float quality_threshold = 0.5f;         // Minimum quality for processing

        // Target performance
        float target_processing_time_ms = 200.0f; // Target total processing time
        float landmark_target_ms = 50.0f;       // Target landmark extraction time
        float reconstruction_target_ms = 100.0f; // Target 3D reconstruction time
        float matching_target_ms = 50.0f;       // Target matching time

        bool validate() const {
            return max_threads > 0 && max_threads <= 8 &&
                   memory_pool_size_mb > 0 && memory_pool_size_mb <= 512 &&
                   target_processing_time_ms > 0 && quality_threshold >= 0;
        }
    };

    /**
     * @brief Performance metrics tracking
     */
    struct PerformanceMetrics {
        // Timing metrics
        double total_processing_time_ms = 0.0;
        double landmark_extraction_time_ms = 0.0;
        double reconstruction_time_ms = 0.0;
        double matching_time_ms = 0.0;
        double overhead_time_ms = 0.0;

        // Throughput metrics
        double fps = 0.0;
        size_t frames_processed = 0;
        size_t total_operations = 0;

        // Resource utilization
        float cpu_utilization = 0.0f;
        float memory_usage_mb = 0.0f;
        float cache_hit_rate = 0.0f;

        // Optimization effectiveness
        float neon_acceleration_factor = 1.0f;
        float threading_efficiency = 1.0f;
        float quality_gating_savings = 0.0f;

        std::chrono::system_clock::time_point measurement_start;
        std::chrono::system_clock::time_point measurement_end;

        void reset() {
            *this = PerformanceMetrics{};
            measurement_start = std::chrono::system_clock::now();
        }

        void finalize() {
            measurement_end = std::chrono::system_clock::now();
            auto duration = measurement_end - measurement_start;
            auto duration_ms = std::chrono::duration<double, std::milli>(duration).count();

            if (frames_processed > 0) {
                fps = (frames_processed * 1000.0) / duration_ms;
            }
        }
    };

private:
    OptimizationConfig config_;
    PerformanceMetrics metrics_;
    std::string last_error_;

    // Memory pools for efficient allocation
    std::vector<std::vector<float>> float_pool_;
    std::vector<cv::Mat> matrix_pool_;
    std::mutex pool_mutex_;

    // Thread pool for parallel processing
    std::vector<std::thread> thread_pool_;
    std::atomic<bool> shutdown_threads_{false};

public:
    explicit ARM64PerformanceOptimizer(const OptimizationConfig& config = OptimizationConfig{})
        : config_(config) {

        if (!config_.validate()) {
            UNLOOK_LOG_WARN("Invalid ARM64 optimization configuration");
        }

        initializeOptimizations();
    }

    ~ARM64PerformanceOptimizer() {
        shutdown_threads_ = true;
        for (auto& thread : thread_pool_) {
            if (thread.joinable()) {
                thread.join();
            }
        }
    }

    /**
     * @brief Optimized 2D to 3D landmark conversion using NEON SIMD
     */
    FaceResultCode optimizedLandmark2DTo3D(const std::vector<LandmarkPoint2D>& landmarks_2d,
                                          const cv::Mat& depth_map,
                                          const cv::Mat& camera_matrix,
                                          std::vector<LandmarkPoint3D>& landmarks_3d) {
        auto start_time = std::chrono::high_resolution_clock::now();

        try {
            landmarks_3d.clear();
            landmarks_3d.reserve(landmarks_2d.size());

#ifdef __ARM_NEON
            // Extract camera parameters
            float fx = static_cast<float>(camera_matrix.at<double>(0, 0));
            float fy = static_cast<float>(camera_matrix.at<double>(1, 1));
            float cx = static_cast<float>(camera_matrix.at<double>(0, 2));
            float cy = static_cast<float>(camera_matrix.at<double>(1, 2));

            // NEON SIMD constants
            float32x4_t fx_vec = vdupq_n_f32(fx);
            float32x4_t fy_vec = vdupq_n_f32(fy);
            float32x4_t cx_vec = vdupq_n_f32(cx);
            float32x4_t cy_vec = vdupq_n_f32(cy);

            // Process landmarks in blocks of 4 using NEON
            size_t block_count = landmarks_2d.size() / 4;
            size_t processed = 0;

            for (size_t block = 0; block < block_count; ++block) {
                // Load 4 landmark coordinates
                float x_coords[4], y_coords[4], depths[4];

                for (int i = 0; i < 4; ++i) {
                    size_t idx = block * 4 + i;
                    x_coords[i] = landmarks_2d[idx].x;
                    y_coords[i] = landmarks_2d[idx].y;

                    // Sample depth with bounds checking
                    int x = static_cast<int>(std::round(x_coords[i]));
                    int y = static_cast<int>(std::round(y_coords[i]));

                    if (x >= 0 && x < depth_map.cols && y >= 0 && y < depth_map.rows) {
                        depths[i] = depth_map.at<float>(y, x);
                    } else {
                        depths[i] = 0.0f;
                    }
                }

                // Load into NEON vectors
                float32x4_t x_vec = vld1q_f32(x_coords);
                float32x4_t y_vec = vld1q_f32(y_coords);
                float32x4_t depth_vec = vld1q_f32(depths);

                // Vectorized 3D coordinate computation
                // x_3d = (x - cx) * depth / fx
                float32x4_t x_3d = vmulq_f32(vsubq_f32(x_vec, cx_vec), vdivq_f32(depth_vec, fx_vec));

                // y_3d = (y - cy) * depth / fy
                float32x4_t y_3d = vmulq_f32(vsubq_f32(y_vec, cy_vec), vdivq_f32(depth_vec, fy_vec));

                // Store results
                float x_3d_results[4], y_3d_results[4];
                vst1q_f32(x_3d_results, x_3d);
                vst1q_f32(y_3d_results, y_3d);

                for (int i = 0; i < 4; ++i) {
                    size_t idx = block * 4 + i;
                    float confidence = landmarks_2d[idx].confidence;
                    float depth_quality = (depths[i] > 0) ? 1.0f : 0.0f;

                    landmarks_3d.emplace_back(x_3d_results[i], y_3d_results[i], depths[i],
                                            confidence, depth_quality);
                    processed++;
                }
            }

            // Process remaining landmarks (non-vectorized)
            for (size_t i = processed; i < landmarks_2d.size(); ++i) {
                const auto& point_2d = landmarks_2d[i];
                int x = static_cast<int>(std::round(point_2d.x));
                int y = static_cast<int>(std::round(point_2d.y));

                if (x >= 0 && x < depth_map.cols && y >= 0 && y < depth_map.rows) {
                    float depth = depth_map.at<float>(y, x);
                    if (depth > 0) {
                        float x_3d = (point_2d.x - cx) * depth / fx;
                        float y_3d = (point_2d.y - cy) * depth / fy;

                        landmarks_3d.emplace_back(x_3d, y_3d, depth, point_2d.confidence, 1.0f);
                    } else {
                        landmarks_3d.emplace_back(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
                    }
                } else {
                    landmarks_3d.emplace_back(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
                }
            }

#else
            // Fallback non-NEON implementation
            float fx = static_cast<float>(camera_matrix.at<double>(0, 0));
            float fy = static_cast<float>(camera_matrix.at<double>(1, 1));
            float cx = static_cast<float>(camera_matrix.at<double>(0, 2));
            float cy = static_cast<float>(camera_matrix.at<double>(1, 2));

            for (const auto& point_2d : landmarks_2d) {
                int x = static_cast<int>(std::round(point_2d.x));
                int y = static_cast<int>(std::round(point_2d.y));

                if (x >= 0 && x < depth_map.cols && y >= 0 && y < depth_map.rows) {
                    float depth = depth_map.at<float>(y, x);
                    if (depth > 0) {
                        float x_3d = (point_2d.x - cx) * depth / fx;
                        float y_3d = (point_2d.y - cy) * depth / fy;

                        landmarks_3d.emplace_back(x_3d, y_3d, depth, point_2d.confidence, 1.0f);
                    } else {
                        landmarks_3d.emplace_back(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
                    }
                } else {
                    landmarks_3d.emplace_back(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
                }
            }
#endif

            auto end_time = std::chrono::high_resolution_clock::now();
            metrics_.landmark_extraction_time_ms +=
                std::chrono::duration<double, std::milli>(end_time - start_time).count();

            return FaceResultCode::SUCCESS;

        } catch (const std::exception& e) {
            last_error_ = "Exception in optimized landmark conversion: " + std::string(e.what());
            return FaceResultCode::ERROR_LANDMARK_DETECTION_FAILED;
        }
    }

    /**
     * @brief Optimized template matching using NEON SIMD
     */
    FaceResultCode optimizedTemplateMatching(const std::vector<float>& probe_features,
                                           const std::vector<float>& reference_features,
                                           float& similarity_score) {
        auto start_time = std::chrono::high_resolution_clock::now();

        try {
            if (probe_features.size() != reference_features.size() || probe_features.empty()) {
                last_error_ = "Feature vector size mismatch or empty";
                return FaceResultCode::ERROR_SIMILARITY_COMPUTATION_FAILED;
            }

#ifdef __ARM_NEON
            size_t size = probe_features.size();
            size_t neon_size = (size / 4) * 4;

            // Compute dot product using NEON
            float32x4_t dot_sum = vdupq_n_f32(0.0f);
            float32x4_t norm1_sum = vdupq_n_f32(0.0f);
            float32x4_t norm2_sum = vdupq_n_f32(0.0f);

            for (size_t i = 0; i < neon_size; i += 4) {
                float32x4_t v1 = vld1q_f32(&probe_features[i]);
                float32x4_t v2 = vld1q_f32(&reference_features[i]);

                // Dot product accumulation
                dot_sum = vfmaq_f32(dot_sum, v1, v2);

                // Norm accumulation
                norm1_sum = vfmaq_f32(norm1_sum, v1, v1);
                norm2_sum = vfmaq_f32(norm2_sum, v2, v2);
            }

            // Sum the 4 elements in each vector
            float dot_product = vaddvq_f32(dot_sum);
            float norm1_sq = vaddvq_f32(norm1_sum);
            float norm2_sq = vaddvq_f32(norm2_sum);

            // Handle remaining elements
            for (size_t i = neon_size; i < size; ++i) {
                dot_product += probe_features[i] * reference_features[i];
                norm1_sq += probe_features[i] * probe_features[i];
                norm2_sq += reference_features[i] * reference_features[i];
            }

            // Compute cosine similarity
            float norm1 = std::sqrt(norm1_sq);
            float norm2 = std::sqrt(norm2_sq);

            if (norm1 > 0 && norm2 > 0) {
                similarity_score = dot_product / (norm1 * norm2);
            } else {
                similarity_score = 0.0f;
            }

            // Record NEON acceleration benefit
            metrics_.neon_acceleration_factor = 3.5f; // Typical NEON speedup

#else
            // Fallback implementation
            float dot_product = std::inner_product(probe_features.begin(), probe_features.end(),
                                                  reference_features.begin(), 0.0f);

            float norm1_sq = std::inner_product(probe_features.begin(), probe_features.end(),
                                               probe_features.begin(), 0.0f);
            float norm2_sq = std::inner_product(reference_features.begin(), reference_features.end(),
                                               reference_features.begin(), 0.0f);

            float norm1 = std::sqrt(norm1_sq);
            float norm2 = std::sqrt(norm2_sq);

            if (norm1 > 0 && norm2 > 0) {
                similarity_score = dot_product / (norm1 * norm2);
            } else {
                similarity_score = 0.0f;
            }

            metrics_.neon_acceleration_factor = 1.0f; // No acceleration
#endif

            similarity_score = std::max(0.0f, std::min(1.0f, similarity_score));

            auto end_time = std::chrono::high_resolution_clock::now();
            metrics_.matching_time_ms +=
                std::chrono::duration<double, std::milli>(end_time - start_time).count();

            return FaceResultCode::SUCCESS;

        } catch (const std::exception& e) {
            last_error_ = "Exception in optimized template matching: " + std::string(e.what());
            return FaceResultCode::ERROR_SIMILARITY_COMPUTATION_FAILED;
        }
    }

    /**
     * @brief Parallel point cloud processing for 3D reconstruction
     */
    FaceResultCode optimizedPointCloudProcessing(const stereo::PointCloud& input_cloud,
                                                stereo::PointCloud& processed_cloud) {
        auto start_time = std::chrono::high_resolution_clock::now();

        try {
            processed_cloud = input_cloud;

            if (processed_cloud.points.empty()) {
                return FaceResultCode::SUCCESS;
            }

            // Quality gating - skip processing for low-quality point clouds
            if (config_.enable_quality_gating) {
                float quality_estimate = estimatePointCloudQuality(processed_cloud);
                if (quality_estimate < config_.quality_threshold) {
                    metrics_.quality_gating_savings += 50.0f; // Estimated time saved
                    return FaceResultCode::SUCCESS;
                }
            }

            if (config_.enable_parallel_processing && config_.max_threads > 1) {
                // Parallel processing using thread pool
                return parallelPointCloudFiltering(processed_cloud);
            } else {
                // Sequential processing
                return sequentialPointCloudFiltering(processed_cloud);
            }

        } catch (const std::exception& e) {
            last_error_ = "Exception in optimized point cloud processing: " + std::string(e.what());
            return FaceResultCode::ERROR_STEREO_RECONSTRUCTION_FAILED;
        }
    }

    /**
     * @brief Memory-optimized buffer management
     */
    std::vector<float> getOptimizedBuffer(size_t size) {
        std::lock_guard<std::mutex> lock(pool_mutex_);

        // Try to reuse existing buffer
        for (auto it = float_pool_.begin(); it != float_pool_.end(); ++it) {
            if (it->size() >= size) {
                std::vector<float> buffer = std::move(*it);
                float_pool_.erase(it);
                buffer.resize(size);
                return buffer;
            }
        }

        // Allocate new buffer if none available
        return std::vector<float>(size);
    }

    /**
     * @brief Return buffer to pool for reuse
     */
    void returnOptimizedBuffer(std::vector<float>&& buffer) {
        std::lock_guard<std::mutex> lock(pool_mutex_);

        // Limit pool size to prevent excessive memory usage
        if (float_pool_.size() < 10) {
            float_pool_.push_back(std::move(buffer));
        }
    }

    /**
     * @brief Fast quality assessment for early termination
     */
    bool isQualitySufficientForProcessing(const cv::Mat& image,
                                        const FacialLandmarks& landmarks) {
        if (!config_.enable_quality_gating) {
            return true;
        }

        // Fast quality checks
        if (landmarks.overall_confidence < config_.quality_threshold) {
            return false;
        }

        if (image.empty() || image.rows < 100 || image.cols < 100) {
            return false;
        }

        // Quick sharpness check using reduced resolution
        cv::Mat small_image;
        cv::resize(image, small_image, cv::Size(100, 100));

        cv::Mat gray;
        if (small_image.channels() == 3) {
            cv::cvtColor(small_image, gray, cv::COLOR_BGR2GRAY);
        } else {
            gray = small_image;
        }

        cv::Mat laplacian;
        cv::Laplacian(gray, laplacian, CV_64F);
        cv::Scalar mean, stddev;
        cv::meanStdDev(laplacian, mean, stddev);

        double variance = stddev[0] * stddev[0];
        float sharpness = static_cast<float>(variance / 100.0); // Normalized

        return sharpness >= config_.quality_threshold;
    }

    /**
     * @brief Get current performance metrics
     */
    PerformanceMetrics getPerformanceMetrics() const {
        return metrics_;
    }

    /**
     * @brief Reset performance metrics
     */
    void resetPerformanceMetrics() {
        metrics_.reset();
    }

    /**
     * @brief Benchmark and optimize for current hardware
     */
    FaceResultCode benchmarkAndOptimize() {
        UNLOOK_LOG_INFO("Starting ARM64 performance benchmark...");

        auto start_time = std::chrono::high_resolution_clock::now();

        try {
            // Test NEON SIMD performance
            testNEONPerformance();

            // Test threading performance
            testThreadingPerformance();

            // Test memory operations
            testMemoryPerformance();

            // Adjust configuration based on results
            optimizeConfiguration();

            auto end_time = std::chrono::high_resolution_clock::now();
            double benchmark_time = std::chrono::duration<double, std::milli>(end_time - start_time).count();

            UNLOOK_LOG_INFO("ARM64 benchmark completed in {:.2f}ms", benchmark_time);
            UNLOOK_LOG_INFO("NEON acceleration factor: {:.2f}x", metrics_.neon_acceleration_factor);
            UNLOOK_LOG_INFO("Threading efficiency: {:.2f}", metrics_.threading_efficiency);

            return FaceResultCode::SUCCESS;

        } catch (const std::exception& e) {
            last_error_ = "Exception during benchmark: " + std::string(e.what());
            return FaceResultCode::ERROR_TEMPLATE_INVALID;
        }
    }

    /**
     * @brief Set optimization configuration
     */
    void setConfiguration(const OptimizationConfig& config) {
        config_ = config;
    }

    /**
     * @brief Get current configuration
     */
    OptimizationConfig getConfiguration() const {
        return config_;
    }

    /**
     * @brief Get last error message
     */
    std::string getLastError() const {
        return last_error_;
    }

private:
    /**
     * @brief Initialize optimization subsystems
     */
    void initializeOptimizations() {
        // Initialize memory pools
        if (config_.enable_memory_pooling) {
            float_pool_.reserve(10);
            matrix_pool_.reserve(10);
        }

        // Initialize thread pool
        if (config_.enable_parallel_processing) {
            thread_pool_.reserve(config_.max_threads);
        }

        // Set CPU affinity if enabled
        if (config_.enable_cpu_affinity) {
            setCPUAffinity();
        }

        UNLOOK_LOG_INFO("ARM64 Performance Optimizer initialized with {} threads, NEON: {}",
                        config_.max_threads,
                        config_.enable_neon_optimizations ? "enabled" : "disabled");
    }

    /**
     * @brief Set CPU affinity for optimal performance
     */
    void setCPUAffinity() {
#ifdef __linux__
        // Set main thread to performance cores (typically cores 4-7 on CM5)
        cpu_set_t cpuset;
        CPU_ZERO(&cpuset);

        // For CM5 with Cortex-A76 cores, prefer cores 4-7
        for (int i = 4; i < std::min(8, config_.max_threads + 4); ++i) {
            CPU_SET(i, &cpuset);
        }

        pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset);
#endif
    }

    /**
     * @brief Test NEON SIMD performance
     */
    void testNEONPerformance() {
#ifdef __ARM_NEON
        const size_t test_size = 1024;
        std::vector<float> vec1(test_size, 1.0f);
        std::vector<float> vec2(test_size, 2.0f);

        auto start_neon = std::chrono::high_resolution_clock::now();

        // NEON dot product test
        float32x4_t sum = vdupq_n_f32(0.0f);
        for (size_t i = 0; i < test_size; i += 4) {
            float32x4_t v1 = vld1q_f32(&vec1[i]);
            float32x4_t v2 = vld1q_f32(&vec2[i]);
            sum = vfmaq_f32(sum, v1, v2);
        }
        float neon_result = vaddvq_f32(sum);

        auto end_neon = std::chrono::high_resolution_clock::now();
        double neon_time = std::chrono::duration<double, std::micro>(end_neon - start_neon).count();

        // Scalar comparison
        auto start_scalar = std::chrono::high_resolution_clock::now();
        float scalar_result = std::inner_product(vec1.begin(), vec1.end(), vec2.begin(), 0.0f);
        auto end_scalar = std::chrono::high_resolution_clock::now();
        double scalar_time = std::chrono::duration<double, std::micro>(end_scalar - start_scalar).count();

        metrics_.neon_acceleration_factor = static_cast<float>(scalar_time / neon_time);

        UNLOOK_LOG_DEBUG("NEON performance: {:.2f}x speedup (results: NEON={:.1f}, scalar={:.1f})",
                        metrics_.neon_acceleration_factor, neon_result, scalar_result);
#else
        metrics_.neon_acceleration_factor = 1.0f;
        UNLOOK_LOG_WARN("NEON instructions not available on this platform");
#endif
    }

    /**
     * @brief Test threading performance
     */
    void testThreadingPerformance() {
        const size_t work_size = 10000;

        // Single-threaded test
        auto start_single = std::chrono::high_resolution_clock::now();
        double single_result = 0.0;
        for (size_t i = 0; i < work_size; ++i) {
            single_result += std::sin(i * 0.001) * std::cos(i * 0.001);
        }
        auto end_single = std::chrono::high_resolution_clock::now();
        double single_time = std::chrono::duration<double, std::milli>(end_single - start_single).count();

        // Multi-threaded test
        auto start_multi = std::chrono::high_resolution_clock::now();
        std::vector<std::future<double>> futures;

        size_t chunk_size = work_size / config_.max_threads;
        for (int t = 0; t < config_.max_threads; ++t) {
            size_t start_idx = t * chunk_size;
            size_t end_idx = (t == config_.max_threads - 1) ? work_size : (t + 1) * chunk_size;

            futures.push_back(std::async(std::launch::async, [start_idx, end_idx]() {
                double result = 0.0;
                for (size_t i = start_idx; i < end_idx; ++i) {
                    result += std::sin(i * 0.001) * std::cos(i * 0.001);
                }
                return result;
            }));
        }

        double multi_result = 0.0;
        for (auto& future : futures) {
            multi_result += future.get();
        }

        auto end_multi = std::chrono::high_resolution_clock::now();
        double multi_time = std::chrono::duration<double, std::milli>(end_multi - start_multi).count();

        metrics_.threading_efficiency = static_cast<float>(single_time / multi_time) / config_.max_threads;

        UNLOOK_LOG_DEBUG("Threading performance: {:.2f} efficiency with {} threads",
                        metrics_.threading_efficiency, config_.max_threads);
    }

    /**
     * @brief Test memory performance
     */
    void testMemoryPerformance() {
        const size_t alloc_size = 1024 * 1024; // 1MB allocations
        const int alloc_count = 10;

        // Test allocation/deallocation overhead
        auto start_time = std::chrono::high_resolution_clock::now();

        for (int i = 0; i < alloc_count; ++i) {
            auto buffer = getOptimizedBuffer(alloc_size);
            // Simulate some work
            std::fill(buffer.begin(), buffer.end(), static_cast<float>(i));
            returnOptimizedBuffer(std::move(buffer));
        }

        auto end_time = std::chrono::high_resolution_clock::now();
        double memory_time = std::chrono::duration<double, std::milli>(end_time - start_time).count();

        metrics_.cache_hit_rate = float_pool_.empty() ? 0.0f : 0.8f; // Estimate

        UNLOOK_LOG_DEBUG("Memory performance: {:.2f}ms for {} allocations",
                        memory_time, alloc_count);
    }

    /**
     * @brief Optimize configuration based on benchmark results
     */
    void optimizeConfiguration() {
        // Adjust threading based on efficiency
        if (metrics_.threading_efficiency < 0.5f) {
            config_.max_threads = std::max(1, config_.max_threads / 2);
            UNLOOK_LOG_INFO("Reduced thread count to {} due to low efficiency", config_.max_threads);
        }

        // Adjust NEON usage based on acceleration factor
        if (metrics_.neon_acceleration_factor < 1.5f) {
            config_.enable_neon_optimizations = false;
            UNLOOK_LOG_INFO("Disabled NEON optimizations due to low acceleration factor");
        }

        // Adjust targets based on measured performance
        if (metrics_.neon_acceleration_factor > 3.0f && metrics_.threading_efficiency > 0.7f) {
            config_.target_processing_time_ms *= 0.8f; // More aggressive target
            UNLOOK_LOG_INFO("Reduced target processing time to {:.1f}ms", config_.target_processing_time_ms);
        }
    }

    /**
     * @brief Estimate point cloud quality quickly
     */
    float estimatePointCloudQuality(const stereo::PointCloud& point_cloud) {
        if (point_cloud.points.empty()) {
            return 0.0f;
        }

        // Quick density check
        float density_score = std::min(1.0f, point_cloud.points.size() / 5000.0f);

        // Quick confidence check (sample first 100 points)
        float confidence_sum = 0.0f;
        size_t sample_size = std::min(static_cast<size_t>(100), point_cloud.points.size());

        for (size_t i = 0; i < sample_size; ++i) {
            confidence_sum += point_cloud.points[i].confidence;
        }

        float confidence_score = confidence_sum / sample_size;

        return (density_score * 0.6f + confidence_score * 0.4f);
    }

    /**
     * @brief Parallel point cloud filtering
     */
    FaceResultCode parallelPointCloudFiltering(stereo::PointCloud& point_cloud) {
        if (point_cloud.points.size() < 1000) {
            return sequentialPointCloudFiltering(point_cloud);
        }

        try {
            std::vector<std::future<std::vector<stereo::Point3D>>> futures;
            size_t chunk_size = point_cloud.points.size() / config_.max_threads;

            for (int t = 0; t < config_.max_threads; ++t) {
                size_t start_idx = t * chunk_size;
                size_t end_idx = (t == config_.max_threads - 1) ?
                                point_cloud.points.size() : (t + 1) * chunk_size;

                futures.push_back(std::async(std::launch::async,
                    [this, &point_cloud, start_idx, end_idx]() {
                        return filterPointChunk(point_cloud.points, start_idx, end_idx);
                    }));
            }

            // Collect results
            std::vector<stereo::Point3D> filtered_points;
            for (auto& future : futures) {
                auto chunk_result = future.get();
                filtered_points.insert(filtered_points.end(),
                                     chunk_result.begin(), chunk_result.end());
            }

            point_cloud.points = std::move(filtered_points);
            point_cloud.isOrganized = false;

            return FaceResultCode::SUCCESS;

        } catch (const std::exception& e) {
            last_error_ = "Exception in parallel filtering: " + std::string(e.what());
            return FaceResultCode::ERROR_STEREO_RECONSTRUCTION_FAILED;
        }
    }

    /**
     * @brief Sequential point cloud filtering
     */
    FaceResultCode sequentialPointCloudFiltering(stereo::PointCloud& point_cloud) {
        std::vector<stereo::Point3D> filtered_points =
            filterPointChunk(point_cloud.points, 0, point_cloud.points.size());

        point_cloud.points = std::move(filtered_points);
        point_cloud.isOrganized = false;

        return FaceResultCode::SUCCESS;
    }

    /**
     * @brief Filter a chunk of points (used by parallel processing)
     */
    std::vector<stereo::Point3D> filterPointChunk(const std::vector<stereo::Point3D>& points,
                                                  size_t start_idx, size_t end_idx) {
        std::vector<stereo::Point3D> filtered_chunk;
        filtered_chunk.reserve(end_idx - start_idx);

        for (size_t i = start_idx; i < end_idx; ++i) {
            const auto& point = points[i];

            // Apply simple filters
            if (point.z >= 200.0f && point.z <= 800.0f && // Depth range
                point.confidence >= 0.3f) {                // Confidence threshold
                filtered_chunk.push_back(point);
            }
        }

        return filtered_chunk;
    }
};

} // namespace face
} // namespace unlook