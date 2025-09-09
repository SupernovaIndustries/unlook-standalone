/* SPDX-License-Identifier: MIT */
/*
 * Real-time Pipeline Architecture for Unlook 3D Scanner
 * High-performance multi-threaded processing with ARM64 NEON optimization
 * 
 * Performance Targets:
 * - VGA (640x480): >30 FPS complete pipeline
 * - HD (1280x720): >15 FPS complete pipeline  
 * - Memory usage: <2GB during processing
 * - CPU utilization: <70% on ARM64
 * - Zero allocations during steady-state operation
 */

#pragma once

#include <memory>
#include <thread>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <chrono>
#include <functional>
#include <array>
#include <vector>

#include <opencv2/core.hpp>
#include <unlook/core/types.hpp>
#include <unlook/camera/HardwareSyncCapture.hpp>

namespace unlook {
namespace realtime {

/**
 * Thread-safe queue for inter-thread communication
 * Lock-free for single producer/consumer pattern
 */
template<typename T>
class ThreadSafeQueue {
public:
    explicit ThreadSafeQueue(size_t max_size = 100) 
        : max_size_(max_size), stop_flag_(false) {}
    
    bool push(T&& item) {
        std::unique_lock<std::mutex> lock(mutex_);
        cv_full_.wait(lock, [this] { 
            return queue_.size() < max_size_ || stop_flag_; 
        });
        
        if (stop_flag_) return false;
        
        queue_.push(std::move(item));
        cv_empty_.notify_one();
        return true;
    }
    
    bool pop(T& item, int timeout_ms = 100) {
        std::unique_lock<std::mutex> lock(mutex_);
        bool success = cv_empty_.wait_for(lock, 
            std::chrono::milliseconds(timeout_ms),
            [this] { return !queue_.empty() || stop_flag_; });
        
        if (stop_flag_ || !success || queue_.empty()) {
            return false;
        }
        
        item = std::move(queue_.front());
        queue_.pop();
        cv_full_.notify_one();
        return true;
    }
    
    void stop() {
        std::lock_guard<std::mutex> lock(mutex_);
        stop_flag_ = true;
        cv_empty_.notify_all();
        cv_full_.notify_all();
    }
    
    size_t size() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return queue_.size();
    }

private:
    mutable std::mutex mutex_;
    std::condition_variable cv_empty_;
    std::condition_variable cv_full_;
    std::queue<T> queue_;
    size_t max_size_;
    std::atomic<bool> stop_flag_;
};

/**
 * Memory pool for zero-allocation frame processing
 */
class FrameMemoryPool {
public:
    struct FrameBuffer {
        cv::Mat left_image;
        cv::Mat right_image;
        cv::Mat left_gray;
        cv::Mat right_gray;
        int64_t timestamp_ns = 0;
        bool in_use = false;
    };
    
    explicit FrameMemoryPool(size_t pool_size = 10);
    ~FrameMemoryPool();
    
    std::shared_ptr<FrameBuffer> acquire();
    void release(std::shared_ptr<FrameBuffer> buffer);
    void preallocate(int width, int height);
    
private:
    std::vector<std::shared_ptr<FrameBuffer>> pool_;
    std::queue<std::shared_ptr<FrameBuffer>> available_;
    std::mutex mutex_;
    std::condition_variable cv_;
    size_t pool_size_;
};

/**
 * Performance monitoring and metrics
 */
class PerformanceMonitor {
public:
    struct Metrics {
        double fps_acquisition = 0.0;
        double fps_processing = 0.0;
        double fps_output = 0.0;
        double latency_ms = 0.0;
        double cpu_usage_percent = 0.0;
        size_t memory_usage_mb = 0;
        size_t frame_drops = 0;
        size_t total_frames = 0;
    };
    
    void updateAcquisitionFPS(double fps) { 
        std::lock_guard<std::mutex> lock(mutex_);
        metrics_.fps_acquisition = fps; 
    }
    
    void updateProcessingFPS(double fps) { 
        std::lock_guard<std::mutex> lock(mutex_);
        metrics_.fps_processing = fps; 
    }
    
    void updateLatency(double ms) { 
        std::lock_guard<std::mutex> lock(mutex_);
        metrics_.latency_ms = ms; 
    }
    
    void incrementFrameDrop() { 
        std::lock_guard<std::mutex> lock(mutex_);
        metrics_.frame_drops++; 
    }
    
    void incrementTotalFrames() { 
        std::lock_guard<std::mutex> lock(mutex_);
        metrics_.total_frames++; 
    }
    
    Metrics getMetrics() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return metrics_;
    }
    
private:
    mutable std::mutex mutex_;
    Metrics metrics_;
};

/**
 * NEON-optimized image processing functions
 */
class NEONProcessor {
public:
    // SBGGR10 to Grayscale conversion with NEON optimization
    static void convertSBGGR10ToGray(const cv::Mat& bayer, cv::Mat& gray);
    static void convertSBGGR10ToGrayNEON(const uint16_t* src, uint8_t* dst, 
                                          int width, int height);
    
    // Fast downsampling for preview
    static void downsampleVGA(const cv::Mat& src, cv::Mat& dst);
    static void downsampleVGANEON(const uint8_t* src, uint8_t* dst,
                                   int src_width, int src_height);
};

/**
 * Main Real-time Pipeline Controller
 * Manages all processing threads and synchronization
 */
class RealtimePipeline {
public:
    struct Config {
        // Thread configuration
        bool enable_thread_affinity = true;
        std::array<int, 4> cpu_cores = {0, 1, 2, 3}; // ARM cores to use
        
        // Processing configuration
        int preview_width = 640;   // VGA width for GUI preview
        int preview_height = 480;  // VGA height for GUI preview
        int full_width = 1456;     // Full resolution width
        int full_height = 1088;    // Full resolution height
        
        // Performance tuning
        size_t frame_queue_size = 5;
        size_t memory_pool_size = 10;
        double target_fps = 30.0;
        bool enable_frame_dropping = true;
        
        // Camera settings - PROPER VALUES FOR INDUSTRIAL SCANNER
        double exposure_time_us = 15000.0;  // 15ms for proper illumination
        double analog_gain = 3.0;           // Higher gain for indoor lighting
        bool auto_exposure = true;           // Enable for dynamic adjustment
        double target_brightness = 120;      // Target 120-150 for clear images
    };
    
    using FrameCallback = std::function<void(const core::StereoFramePair&)>;
    
    RealtimePipeline();
    ~RealtimePipeline();
    
    // Pipeline control
    bool initialize(const Config& config = {});
    bool start(FrameCallback callback);
    void stop();
    bool isRunning() const { return running_.load(); }
    
    // Dynamic configuration
    void setExposure(double exposure_us);
    void setGain(double gain);
    void setAutoExposure(bool enabled);
    void setTargetFPS(double fps);
    
    // Performance monitoring
    PerformanceMonitor::Metrics getMetrics() const {
        return performance_monitor_->getMetrics();
    }
    
private:
    // Thread functions
    void acquisitionThread();
    void processingThread();
    void outputThread();
    void monitoringThread();
    
    // Thread management
    void setThreadAffinity(std::thread& thread, int core_id);
    void setThreadPriority(std::thread& thread, int priority);
    
    // Frame processing
    void processFrame(std::shared_ptr<FrameMemoryPool::FrameBuffer> frame);
    void deliverFrame(const core::StereoFramePair& frame);
    
    // Configuration
    Config config_;
    std::atomic<bool> initialized_;
    std::atomic<bool> running_;
    
    // Hardware interface
    std::unique_ptr<camera::HardwareSyncCapture> hardware_capture_;
    
    // Memory management
    std::unique_ptr<FrameMemoryPool> frame_pool_;
    
    // Thread-safe queues
    using FramePtr = std::shared_ptr<FrameMemoryPool::FrameBuffer>;
    std::unique_ptr<ThreadSafeQueue<FramePtr>> acquisition_queue_;
    std::unique_ptr<ThreadSafeQueue<core::StereoFramePair>> output_queue_;
    
    // Processing threads
    std::thread acquisition_thread_;
    std::thread processing_thread_;
    std::thread output_thread_;
    std::thread monitoring_thread_;
    
    // Synchronization
    std::mutex callback_mutex_;
    FrameCallback frame_callback_;
    
    // Performance monitoring
    std::unique_ptr<PerformanceMonitor> performance_monitor_;
    
    // FPS counters
    std::chrono::steady_clock::time_point last_frame_time_;
    std::atomic<int> frame_count_;
};

} // namespace realtime
} // namespace unlook