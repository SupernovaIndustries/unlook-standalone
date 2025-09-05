/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * High-Performance ARM64 NEON Bayer Processing
 * Optimized for Unlook 3D Scanner Real-time Pipeline
 */

#pragma once

#include <cstdint>
#include <opencv2/core.hpp>

namespace unlook {
namespace realtime {

/**
 * High-performance SBGGR10 unpacking with NEON vectorization
 * Unpacks 10-bit packed Bayer data to 16-bit for processing
 * 
 * @param src Input SBGGR10 packed data
 * @param dst Output 16-bit unpacked data
 * @param width Image width in pixels
 * @param height Image height in pixels
 * 
 * Performance targets:
 * - VGA (640x480): <5ms
 * - HD (1280x720): <15ms
 */
void unpackSBGGR10_NEON_Optimized(const uint8_t* __restrict src, 
                                  uint16_t* __restrict dst,
                                  const int width, const int height);

/**
 * NEON-optimized bilinear Bayer demosaicing
 * Fast demosaicing for real-time preview
 * 
 * @param bayer Input 16-bit Bayer pattern image
 * @param rgb Output RGB image
 */
void demosaicBilinearNEON(const cv::Mat& bayer, cv::Mat& rgb);

/**
 * Complete optimized pipeline: SBGGR10 -> RGB/Gray
 * Zero-allocation processing with thread-local buffers
 * 
 * @param raw_data Input SBGGR10 packed data
 * @param width Image width
 * @param height Image height  
 * @param output_grayscale True for grayscale output (faster)
 * @return Processed image (RGB or grayscale)
 * 
 * Performance:
 * - Uses thread-local static buffers to avoid allocations
 * - NEON SIMD acceleration throughout
 * - Optimized for ARM64 cache architecture
 */
cv::Mat processRawFrameOptimized(const uint8_t* raw_data, 
                                 int width, int height,
                                 bool output_grayscale = true);

/**
 * Performance profiling utilities
 */
class BayerProfiler {
public:
    struct Stats {
        double unpack_ms = 0.0;
        double demosaic_ms = 0.0;
        double total_ms = 0.0;
        uint64_t frames_processed = 0;
    };
    
    static Stats getStats();
    static void reset();
    static void enableProfiling(bool enable);
    
private:
    static thread_local Stats stats_;
    static thread_local bool profiling_enabled_;
};

} // namespace realtime
} // namespace unlook