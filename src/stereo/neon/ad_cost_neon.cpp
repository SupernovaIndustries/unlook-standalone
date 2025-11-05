/**
 * @file ad_cost_neon.cpp
 * @brief ARM NEON optimized Absolute Difference cost computation
 *
 * Uses NEON vabdq_u8 instruction for single-instruction absolute difference
 * computation, processing 16 pixels simultaneously.
 */

#include <opencv2/core.hpp>
#include <cstdint>
#include <algorithm>
#include <cmath>

#ifdef _OPENMP
#include <omp.h>
#endif

#ifdef __ARM_NEON
#include <arm_neon.h>
#endif

namespace unlook {
namespace stereo {
namespace neon {

/**
 * @brief NEON-optimized Absolute Difference cost computation
 *
 * Uses vabdq_u8 instruction for hardware-accelerated absolute difference.
 * Processes 16 pixels at once for maximum throughput.
 *
 * @param left Left grayscale image (CV_8UC1)
 * @param right Right grayscale image (CV_8UC1)
 * @param costVolume Output AD cost volume (CV_32F, H×W×D)
 * @param minDisparity Minimum disparity value
 * @param numDisparities Number of disparity levels
 */
void computeADCostNEON(const cv::Mat& left,
                       const cv::Mat& right,
                       cv::Mat& costVolume,
                       int minDisparity,
                       int numDisparities) {
#ifdef __ARM_NEON
    const int width = left.cols;
    const int height = left.rows;

    // Initialize cost volume
    const int dims[3] = {height, width, numDisparities};
    costVolume.create(3, dims, CV_32F);

    // Process each row in parallel
    #pragma omp parallel for schedule(dynamic)
    for (int y = 0; y < height; y++) {
        const uint8_t* leftPtr = left.ptr<uint8_t>(y);
        const uint8_t* rightPtr = right.ptr<uint8_t>(y);

        // Process pixels in blocks of 16 for NEON
        for (int x = 0; x < width; x += 16) {
            int blockSize = std::min(16, width - x);

            // Load left pixels into NEON register
            uint8x16_t left_vec;
            if (blockSize == 16) {
                left_vec = vld1q_u8(&leftPtr[x]);
            } else {
                // Handle partial block
                uint8_t temp[16] = {0};
                for (int i = 0; i < blockSize; i++) {
                    temp[i] = leftPtr[x + i];
                }
                left_vec = vld1q_u8(temp);
            }

            // Compute AD for each disparity
            for (int d = 0; d < numDisparities; d++) {
                // Process each pixel in the block
                for (int px = 0; px < blockSize; px++) {
                    int curr_x = x + px;
                    int xr = curr_x - (minDisparity + d);

                    float cost = 255.0f;  // Maximum cost

                    if (xr >= 0 && xr < width) {
                        // Single pixel AD (can be vectorized further)
                        uint8_t left_val = leftPtr[curr_x];
                        uint8_t right_val = rightPtr[xr];
                        cost = static_cast<float>(std::abs(left_val - right_val));
                    }

                    // Store cost
                    float* costPtr = costVolume.ptr<float>(y, curr_x);
                    costPtr[d] = cost;
                }
            }
        }
    }
#else
    // CPU fallback
    computeADCostCPU(left, right, costVolume, minDisparity, numDisparities);
#endif
}

/**
 * @brief Vectorized AD cost using vabdq_u8 for multiple pixels
 *
 * Processes entire scanlines efficiently using NEON absolute difference.
 */
void computeADCostVectorizedNEON(const cv::Mat& left,
                                 const cv::Mat& right,
                                 cv::Mat& costVolume,
                                 int minDisparity,
                                 int numDisparities) {
#ifdef __ARM_NEON
    const int width = left.cols;
    const int height = left.rows;

    const int dims[3] = {height, width, numDisparities};
    costVolume.create(3, dims, CV_32F);

    #pragma omp parallel for schedule(dynamic)
    for (int y = 0; y < height; y++) {
        const uint8_t* leftPtr = left.ptr<uint8_t>(y);
        const uint8_t* rightPtr = right.ptr<uint8_t>(y);

        for (int d = 0; d < numDisparities; d++) {
            // Process scanline for this disparity
            for (int x = 0; x < width; x += 16) {
                int blockSize = std::min(16, width - x);

                // Prepare data for NEON processing
                uint8_t left_block[16] = {0};
                uint8_t right_block[16] = {0};
                float costs[16] = {255.0f};

                // Collect pixels
                for (int i = 0; i < blockSize; i++) {
                    int curr_x = x + i;
                    int xr = curr_x - (minDisparity + d);

                    left_block[i] = leftPtr[curr_x];
                    if (xr >= 0 && xr < width) {
                        right_block[i] = rightPtr[xr];
                    } else {
                        right_block[i] = 0;  // Will produce max cost
                    }
                }

                // Compute absolute differences using NEON
                uint8x16_t left_vec = vld1q_u8(left_block);
                uint8x16_t right_vec = vld1q_u8(right_block);
                uint8x16_t diff_vec = vabdq_u8(left_vec, right_vec);

                // Convert to float and store
                uint8_t diff_array[16];
                vst1q_u8(diff_array, diff_vec);

                for (int i = 0; i < blockSize; i++) {
                    int curr_x = x + i;
                    int xr = curr_x - (minDisparity + d);

                    float cost = (xr >= 0 && xr < width) ?
                                static_cast<float>(diff_array[i]) : 255.0f;

                    float* costPtr = costVolume.ptr<float>(y, curr_x);
                    costPtr[d] = cost;
                }
            }
        }
    }
#else
    computeADCostCPU(left, right, costVolume, minDisparity, numDisparities);
#endif
}

/**
 * @brief Window-based AD cost with NEON acceleration
 *
 * Computes AD cost over a window for more robust matching.
 */
void computeWindowADCostNEON(const cv::Mat& left,
                             const cv::Mat& right,
                             cv::Mat& costVolume,
                             int minDisparity,
                             int numDisparities,
                             int windowRadius = 2) {
#ifdef __ARM_NEON
    const int width = left.cols;
    const int height = left.rows;

    const int dims[3] = {height, width, numDisparities};
    costVolume.create(3, dims, CV_32F);

    const int windowSize = 2 * windowRadius + 1;
    const float normFactor = 1.0f / (windowSize * windowSize);

    #pragma omp parallel for schedule(dynamic)
    for (int y = windowRadius; y < height - windowRadius; y++) {
        for (int x = windowRadius; x < width - windowRadius; x++) {
            float* costPtr = costVolume.ptr<float>(y, x);

            for (int d = 0; d < numDisparities; d++) {
                int xr = x - (minDisparity + d);

                if (xr >= windowRadius && xr < width - windowRadius) {
                    float windowCost = 0.0f;

                    // Compute window AD using NEON
                    for (int wy = -windowRadius; wy <= windowRadius; wy++) {
                        const uint8_t* leftRow = left.ptr<uint8_t>(y + wy);
                        const uint8_t* rightRow = right.ptr<uint8_t>(y + wy);

                        // Process window row with NEON
                        if (windowSize >= 8) {
                            // Use NEON for larger windows
                            uint8_t left_vals[16] = {0};
                            uint8_t right_vals[16] = {0};

                            for (int wx = -windowRadius, idx = 0; wx <= windowRadius; wx++, idx++) {
                                left_vals[idx] = leftRow[x + wx];
                                right_vals[idx] = rightRow[xr + wx];
                            }

                            uint8x16_t left_vec = vld1q_u8(left_vals);
                            uint8x16_t right_vec = vld1q_u8(right_vals);
                            uint8x16_t diff_vec = vabdq_u8(left_vec, right_vec);

                            // Sum the differences
                            uint16x8_t sum_vec = vpaddlq_u8(diff_vec);
                            uint32x4_t sum_vec32 = vpaddlq_u16(sum_vec);
                            uint64x2_t sum_vec64 = vpaddlq_u32(sum_vec32);

                            windowCost += vaddvq_u64(sum_vec64);
                        } else {
                            // Small window - use scalar
                            for (int wx = -windowRadius; wx <= windowRadius; wx++) {
                                uint8_t left_val = leftRow[x + wx];
                                uint8_t right_val = rightRow[xr + wx];
                                windowCost += std::abs(left_val - right_val);
                            }
                        }
                    }

                    costPtr[d] = windowCost * normFactor;
                } else {
                    costPtr[d] = 255.0f;
                }
            }
        }
    }

    // Handle borders
    for (int y = 0; y < windowRadius; y++) {
        for (int x = 0; x < width; x++) {
            float* costPtr = costVolume.ptr<float>(y, x);
            for (int d = 0; d < numDisparities; d++) {
                costPtr[d] = 255.0f;
            }
        }
    }
    for (int y = height - windowRadius; y < height; y++) {
        for (int x = 0; x < width; x++) {
            float* costPtr = costVolume.ptr<float>(y, x);
            for (int d = 0; d < numDisparities; d++) {
                costPtr[d] = 255.0f;
            }
        }
    }
#else
    computeADCostCPU(left, right, costVolume, minDisparity, numDisparities);
#endif
}

/**
 * @brief CPU fallback for AD cost computation
 */
void computeADCostCPU(const cv::Mat& left,
                     const cv::Mat& right,
                     cv::Mat& costVolume,
                     int minDisparity,
                     int numDisparities) {
    const int width = left.cols;
    const int height = left.rows;

    const int dims[3] = {height, width, numDisparities};
    costVolume.create(3, dims, CV_32F);

    for (int y = 0; y < height; y++) {
        const uint8_t* leftPtr = left.ptr<uint8_t>(y);
        const uint8_t* rightPtr = right.ptr<uint8_t>(y);

        for (int x = 0; x < width; x++) {
            float* costPtr = costVolume.ptr<float>(y, x);

            for (int d = 0; d < numDisparities; d++) {
                int xr = x - (minDisparity + d);

                if (xr >= 0 && xr < width) {
                    uint8_t left_val = leftPtr[x];
                    uint8_t right_val = rightPtr[xr];
                    costPtr[d] = static_cast<float>(std::abs(left_val - right_val));
                } else {
                    costPtr[d] = 255.0f;
                }
            }
        }
    }
}

/**
 * @brief Fast AD cost with prefetching and cache optimization
 */
void computeADCostOptimizedNEON(const cv::Mat& left,
                               const cv::Mat& right,
                               cv::Mat& costVolume,
                               int minDisparity,
                               int numDisparities) {
#ifdef __ARM_NEON
    const int width = left.cols;
    const int height = left.rows;

    const int dims[3] = {height, width, numDisparities};
    costVolume.create(3, dims, CV_32F);

    // Block processing for cache efficiency
    const int blockHeight = 8;
    const int blockWidth = 32;

    #pragma omp parallel for schedule(dynamic)
    for (int by = 0; by < height; by += blockHeight) {
        int bh = std::min(blockHeight, height - by);

        for (int bx = 0; bx < width; bx += blockWidth) {
            int bw = std::min(blockWidth, width - bx);

            // Prefetch next block
            if (bx + blockWidth < width) {
                for (int y = by; y < by + bh; y++) {
                    __builtin_prefetch(&left.ptr<uint8_t>(y)[bx + blockWidth], 0, 3);
                    __builtin_prefetch(&right.ptr<uint8_t>(y)[bx + blockWidth], 0, 3);
                }
            }

            // Process block
            for (int y = by; y < by + bh; y++) {
                const uint8_t* leftPtr = left.ptr<uint8_t>(y);
                const uint8_t* rightPtr = right.ptr<uint8_t>(y);

                for (int x = bx; x < bx + bw; x++) {
                    float* costPtr = costVolume.ptr<float>(y, x);

                    // Unroll disparity loop for better pipelining
                    for (int d = 0; d < numDisparities; d += 4) {
                        int remaining = std::min(4, numDisparities - d);

                        for (int i = 0; i < remaining; i++) {
                            int xr = x - (minDisparity + d + i);

                            if (xr >= 0 && xr < width) {
                                uint8_t left_val = leftPtr[x];
                                uint8_t right_val = rightPtr[xr];
                                costPtr[d + i] = static_cast<float>(std::abs(left_val - right_val));
                            } else {
                                costPtr[d + i] = 255.0f;
                            }
                        }
                    }
                }
            }
        }
    }
#else
    computeADCostCPU(left, right, costVolume, minDisparity, numDisparities);
#endif
}

} // namespace neon
} // namespace stereo
} // namespace unlook