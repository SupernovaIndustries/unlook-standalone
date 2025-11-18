/**
 * @file RealtimePipeline.hpp
 * @brief High-performance multi-threaded real-time processing pipeline
 * @author Unlook Real-time Pipeline Architect
 * @date 2025-11-18
 *
 * ARCHITECTURE OVERVIEW:
 * - Multi-threaded pipeline with 4 stages (acquisition, processing, filtering, output)
 * - Lock-free queues for inter-thread communication
 * - Memory pools for zero-allocation processing
 * - ARM64 NEON optimizations throughout
 * - Thread affinity for optimal core utilization on RPi5
 *
 * PERFORMANCE TARGETS:
 * - VGA (640x480): >20 FPS complete pipeline
 * - HD (1280x720): >10 FPS complete pipeline
 * - CPU utilization: <80% average on 4 cores
 * - Memory allocations: <100 per second
 */

#pragma once

#include <atomic>
#include <thread>
#include <vector>
#include <memory>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <queue>
#include <array>
#include <opencv2/core.hpp>

namespace unlook {
namespace realtime {

/**
 * Lock-free queue for high-performance inter-thread communication
 */
template<typename T, size_t Size>
class LockFreeQueue {
public:
    LockFreeQueue() : head_(0), tail_(0) {}

    bool push(T&& item) {
        size_t current_tail = tail_.load(std::memory_order_relaxed);
        size_t next_tail = (current_tail + 1) % Size;

        if (next_tail == head_.load(std::memory_order_acquire)) {
            return false; // Queue full
        }

        buffer_[current_tail] = std::move(item);
        tail_.store(next_tail, std::memory_order_release);
        return true;
    }

    bool pop(T& item) {
        size_t current_head = head_.load(std::memory_order_relaxed);

        if (current_head == tail_.load(std::memory_order_acquire)) {
            return false; // Queue empty
        }

        item = std::move(buffer_[current_head]);
        head_.store((current_head + 1) % Size, std::memory_order_release);
        return true;
    }

    bool empty() const {
        return head_.load(std::memory_order_acquire) ==
               tail_.load(std::memory_order_acquire);
    }

    size_t size() const {
        size_t h = head_.load(std::memory_order_acquire);
        size_t t = tail_.load(std::memory_order_acquire);
        return (t >= h) ? (t - h) : (Size - h + t);
    }

private:
    std::array<T, Size> buffer_;
    alignas(64) std::atomic<size_t> head_;  // Cache line aligned
    alignas(64) std::atomic<size_t> tail_;  // Cache line aligned
};

/**
 * Memory pool for zero-allocation processing
 */
class MemoryPool {
public:
    explicit MemoryPool(size_t block_size, size_t num_blocks);

    cv::Mat allocate(int rows, int cols, int type);
    void release(const cv::Mat& mat);

    size_t available_blocks() const { return available_.size(); }
    size_t total_blocks() const { return total_blocks_; }

private:
    struct Block {
        std::vector<uint8_t> data;
        bool in_use = false;
    };

    size_t block_size_;
    size_t total_blocks_;
    std::vector<Block> blocks_;
    LockFreeQueue<size_t, 128> available_;
    std::mutex mutex_;
};

/**
 * Performance metrics for real-time monitoring
 */
struct PerformanceMetrics {
    // Timing metrics (all in milliseconds)
    double acquisition_time = 0;
    double rectification_time = 0;
    double census_time = 0;
    double sgm_time = 0;
    double filtering_time = 0;
    double pointcloud_time = 0;
    double total_time = 0;

    // Throughput metrics
    double fps_current = 0;
    double fps_average = 0;
    size_t frames_processed = 0;

    // Resource metrics
    double cpu_usage_percent = 0;
    size_t memory_usage_mb = 0;
    size_t allocations_per_second = 0;

    // Quality metrics
    double valid_pixels_percent = 0;
    double outliers_removed_percent = 0;
    size_t points_generated = 0;

    // Thread metrics
    std::array<double, 4> core_usage_percent = {0, 0, 0, 0};
    size_t queue_depth_processing = 0;
    size_t queue_depth_filtering = 0;

    void reset() { *this = PerformanceMetrics(); }
};

/**
 * Pipeline frame data structure
 */
struct PipelineFrame {
    // Frame ID and timing
    uint64_t frame_id = 0;
    std::chrono::high_resolution_clock::time_point timestamp;

    // Image data (using memory pool allocations)
    cv::Mat left_raw;
    cv::Mat right_raw;
    cv::Mat left_rectified;
    cv::Mat right_rectified;
    cv::Mat census_left;
    cv::Mat census_right;
    cv::Mat disparity;
    cv::Mat points_3d;

    // Metadata
    bool valid = false;
    std::string error_message;

    // Performance tracking
    std::chrono::high_resolution_clock::time_point stage_start;
    std::chrono::high_resolution_clock::time_point stage_end;
};

/**
 * Pipeline configuration
 */
struct PipelineConfig {
    // Threading configuration
    bool enable_multithreading = true;
    int num_worker_threads = 4;
    bool enable_thread_affinity = true;
    std::array<int, 4> core_affinity = {0, 1, 2, 3};  // RPi5 cores

    // Processing configuration
    bool enable_neon_optimizations = true;
    bool enable_memory_pooling = true;
    bool enable_frame_dropping = true;
    double target_fps = 20.0;  // For VGA

    // Optimization parameters
    int census_block_size = 64;  // Process census in 64x64 blocks
    int sgm_path_threads = 4;    // Parallel SGM paths
    int outlier_grid_size = 16;  // Spatial grid for outlier removal

    // Memory pool configuration
    size_t memory_pool_blocks = 32;
    size_t memory_pool_block_size = 4 * 1024 * 1024;  // 4MB blocks

    // Quality vs Speed trade-offs
    bool enable_statistical_filter = true;
    int statistical_filter_decimation = 2;  // Process every 2nd pixel
    bool enable_border_filter = true;
    int border_margin = 30;  // Reduced from 40
};

/**
 * Main real-time pipeline class
 */
class RealtimePipeline {
public:
    explicit RealtimePipeline(const PipelineConfig& config = PipelineConfig());
    ~RealtimePipeline();

    // Pipeline control
    bool initialize();
    void start();
    void stop();
    bool is_running() const { return running_.load(); }

    // Frame submission
    bool submit_frame(const cv::Mat& left, const cv::Mat& right);

    // Result retrieval
    bool get_latest_pointcloud(cv::Mat& pointcloud);
    bool get_latest_disparity(cv::Mat& disparity);

    // Performance monitoring
    PerformanceMetrics get_metrics() const;
    void reset_metrics();

    // Configuration
    void update_config(const PipelineConfig& config);
    PipelineConfig get_config() const { return config_; }

private:
    // Thread workers
    void acquisition_thread();
    void processing_thread(int thread_id);
    void filtering_thread();
    void output_thread();

    // Processing stages (optimized implementations)
    void rectify_stereo_pair_optimized(PipelineFrame& frame);
    void compute_census_neon(PipelineFrame& frame);
    void compute_sgm_parallel(PipelineFrame& frame);
    void apply_filters_optimized(PipelineFrame& frame);
    void generate_pointcloud_optimized(PipelineFrame& frame);

    // NEON optimized kernels
    void census_transform_neon(const cv::Mat& src, cv::Mat& dst);
    void hamming_distance_neon(const cv::Mat& left, const cv::Mat& right, cv::Mat& cost);
    void statistical_filter_spatial(cv::Mat& points);

    // Thread management
    void set_thread_affinity(std::thread& t, int core_id);
    void monitor_performance();

private:
    // Configuration
    PipelineConfig config_;

    // Thread control
    std::atomic<bool> running_;
    std::atomic<bool> initialized_;

    // Worker threads
    std::thread acquisition_thread_;
    std::vector<std::thread> processing_threads_;
    std::thread filtering_thread_;
    std::thread output_thread_;
    std::thread monitor_thread_;

    // Inter-thread communication (lock-free queues)
    LockFreeQueue<PipelineFrame, 8> acquisition_queue_;
    LockFreeQueue<PipelineFrame, 8> processing_queue_;
    LockFreeQueue<PipelineFrame, 8> filtering_queue_;
    LockFreeQueue<PipelineFrame, 4> output_queue_;

    // Memory management
    std::unique_ptr<MemoryPool> memory_pool_;

    // Performance tracking
    mutable std::mutex metrics_mutex_;
    PerformanceMetrics metrics_;
    std::chrono::high_resolution_clock::time_point pipeline_start_time_;

    // Frame ID generation
    std::atomic<uint64_t> next_frame_id_;

    // Calibration data (cached)
    cv::Mat camera_matrix_left_, camera_matrix_right_;
    cv::Mat dist_coeffs_left_, dist_coeffs_right_;
    cv::Mat R1_, R2_, P1_, P2_, Q_;
    cv::Mat map1_left_, map2_left_;
    cv::Mat map1_right_, map2_right_;
    cv::Size image_size_;
};

} // namespace realtime
} // namespace unlook