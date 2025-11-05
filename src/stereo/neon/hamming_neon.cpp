/**
 * @file hamming_neon.cpp
 * @brief ARM NEON optimized Hamming distance computation
 *
 * Uses NEON POPCOUNT (vcntq_u8) instruction for hardware-accelerated
 * bit counting in Census descriptor matching.
 */

#include <opencv2/core.hpp>
#include <cstdint>
#include <cstring>
#include <algorithm>

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
 * @brief Compute Hamming distance between two 80-bit descriptors
 */
inline int hammingDistance80(uint64_t low1, uint16_t high1,
                            uint64_t low2, uint16_t high2) {
#ifdef __ARM_NEON
    // XOR the descriptors to find differing bits
    uint64_t xor_low = low1 ^ low2;
    uint16_t xor_high = high1 ^ high2;

    // Use NEON POPCOUNT for the 64-bit part
    uint8x8_t bytes_low = vreinterpret_u8_u64(vdup_n_u64(xor_low));
    uint8x8_t popcount_low = vcnt_u8(bytes_low);
    uint32_t count_low = vaddlv_u8(popcount_low);

    // Count bits in the 16-bit part
    uint32_t count_high = __builtin_popcount(xor_high);

    return count_low + count_high;
#else
    // CPU fallback using builtin popcount
    uint64_t xor_low = low1 ^ low2;
    uint16_t xor_high = high1 ^ high2;
    return __builtin_popcountll(xor_low) + __builtin_popcount(xor_high);
#endif
}

/**
 * @brief NEON-optimized Hamming cost volume computation
 *
 * Computes matching costs between left and right Census descriptors
 * using NEON POPCOUNT instructions for maximum performance.
 *
 * @param censusLeft Left census descriptors (CV_64FC2)
 * @param censusRight Right census descriptors (CV_64FC2)
 * @param costVolume Output cost volume (CV_32FC1, H×W×D)
 * @param minDisparity Minimum disparity value
 * @param numDisparities Number of disparity levels
 */
void computeHammingCostNEON(const cv::Mat& censusLeft,
                            const cv::Mat& censusRight,
                            cv::Mat& costVolume,
                            int minDisparity,
                            int numDisparities) {
#ifdef __ARM_NEON
    const int width = censusLeft.cols;
    const int height = censusLeft.rows;

    // Initialize cost volume (3D: height × width × numDisparities)
    const int dims[3] = {height, width, numDisparities};
    costVolume.create(3, dims, CV_32F);

    // Process each row in parallel
    #pragma omp parallel for schedule(dynamic)
    for (int y = 0; y < height; y++) {
        const cv::Vec2d* leftPtr = censusLeft.ptr<cv::Vec2d>(y);
        const cv::Vec2d* rightPtr = censusRight.ptr<cv::Vec2d>(y);

        for (int x = 0; x < width; x++) {
            // Extract left descriptor
            uint64_t left_low, left_high_temp;
            memcpy(&left_low, &leftPtr[x][0], sizeof(uint64_t));
            memcpy(&left_high_temp, &leftPtr[x][1], sizeof(uint64_t));
            uint16_t left_high = static_cast<uint16_t>(left_high_temp);

            // Compute costs for all disparities
            for (int d = 0; d < numDisparities; d++) {
                int xr = x - (minDisparity + d);
                float cost = 80.0f;  // Maximum cost (all bits different)

                if (xr >= 0 && xr < width) {
                    // Extract right descriptor
                    uint64_t right_low, right_high_temp;
                    memcpy(&right_low, &rightPtr[xr][0], sizeof(uint64_t));
                    memcpy(&right_high_temp, &rightPtr[xr][1], sizeof(uint64_t));
                    uint16_t right_high = static_cast<uint16_t>(right_high_temp);

                    // Compute Hamming distance using NEON
                    cost = static_cast<float>(
                        hammingDistance80(left_low, left_high, right_low, right_high)
                    );
                }

                // Store cost in volume
                float* costPtr = costVolume.ptr<float>(y, x);
                costPtr[d] = cost;
            }
        }
    }
#else
    // CPU fallback
    computeHammingCostCPU(censusLeft, censusRight, costVolume, minDisparity, numDisparities);
#endif
}

/**
 * @brief Vectorized Hamming cost computation using NEON
 *
 * Processes multiple disparities simultaneously for better cache usage
 * and SIMD utilization.
 */
void computeHammingCostVectorizedNEON(const cv::Mat& censusLeft,
                                      const cv::Mat& censusRight,
                                      cv::Mat& costVolume,
                                      int minDisparity,
                                      int numDisparities) {
#ifdef __ARM_NEON
    const int width = censusLeft.cols;
    const int height = censusLeft.rows;

    const int dims[3] = {height, width, numDisparities};
    costVolume.create(3, dims, CV_32F);

    // Process in blocks of 4 disparities for NEON float32x4
    const int dispBlock = 4;

    #pragma omp parallel for schedule(dynamic)
    for (int y = 0; y < height; y++) {
        const cv::Vec2d* leftPtr = censusLeft.ptr<cv::Vec2d>(y);
        const cv::Vec2d* rightPtr = censusRight.ptr<cv::Vec2d>(y);

        for (int x = 0; x < width; x++) {
            uint64_t left_low, left_high_temp;
            memcpy(&left_low, &leftPtr[x][0], sizeof(uint64_t));
            memcpy(&left_high_temp, &leftPtr[x][1], sizeof(uint64_t));
            uint16_t left_high = static_cast<uint16_t>(left_high_temp);

            float* costPtr = costVolume.ptr<float>(y, x);

            // Process disparities in blocks of 4
            for (int d = 0; d < numDisparities; d += dispBlock) {
                float32x4_t costs = vdupq_n_f32(80.0f);  // Initialize to max cost
                float cost_array[4] = {80.0f, 80.0f, 80.0f, 80.0f};

                int remaining = std::min(dispBlock, numDisparities - d);

                for (int i = 0; i < remaining; i++) {
                    int xr = x - (minDisparity + d + i);

                    if (xr >= 0 && xr < width) {
                        uint64_t right_low, right_high_temp;
                        memcpy(&right_low, &rightPtr[xr][0], sizeof(uint64_t));
                        memcpy(&right_high_temp, &rightPtr[xr][1], sizeof(uint64_t));
                        uint16_t right_high = static_cast<uint16_t>(right_high_temp);

                        // Compute Hamming distance with NEON POPCOUNT
                        uint64_t xor_low = left_low ^ right_low;
                        uint16_t xor_high = left_high ^ right_high;

                        // NEON POPCOUNT for 64-bit part
                        uint8x8_t bytes = vreinterpret_u8_u64(vdup_n_u64(xor_low));
                        uint8x8_t popcount = vcnt_u8(bytes);
                        uint32_t count = vaddlv_u8(popcount);

                        // Add 16-bit part
                        count += __builtin_popcount(xor_high);

                        cost_array[i] = static_cast<float>(count);
                    }
                }

                // Store costs using NEON store
                costs = vld1q_f32(cost_array);
                vst1q_f32(&costPtr[d], costs);
            }
        }
    }
#else
    computeHammingCostCPU(censusLeft, censusRight, costVolume, minDisparity, numDisparities);
#endif
}

/**
 * @brief CPU fallback for Hamming cost computation
 */
void computeHammingCostCPU(const cv::Mat& censusLeft,
                           const cv::Mat& censusRight,
                           cv::Mat& costVolume,
                           int minDisparity,
                           int numDisparities) {
    const int width = censusLeft.cols;
    const int height = censusLeft.rows;

    const int dims[3] = {height, width, numDisparities};
    costVolume.create(3, dims, CV_32F);

    for (int y = 0; y < height; y++) {
        const cv::Vec2d* leftPtr = censusLeft.ptr<cv::Vec2d>(y);
        const cv::Vec2d* rightPtr = censusRight.ptr<cv::Vec2d>(y);

        for (int x = 0; x < width; x++) {
            uint64_t left_low, left_high_temp;
            memcpy(&left_low, &leftPtr[x][0], sizeof(uint64_t));
            memcpy(&left_high_temp, &leftPtr[x][1], sizeof(uint64_t));
            uint16_t left_high = static_cast<uint16_t>(left_high_temp);

            float* costPtr = costVolume.ptr<float>(y, x);

            for (int d = 0; d < numDisparities; d++) {
                int xr = x - (minDisparity + d);
                float cost = 80.0f;

                if (xr >= 0 && xr < width) {
                    uint64_t right_low, right_high_temp;
                    memcpy(&right_low, &rightPtr[xr][0], sizeof(uint64_t));
                    memcpy(&right_high_temp, &rightPtr[xr][1], sizeof(uint64_t));
                    uint16_t right_high = static_cast<uint16_t>(right_high_temp);

                    uint64_t xor_low = left_low ^ right_low;
                    uint16_t xor_high = left_high ^ right_high;

                    int distance = __builtin_popcountll(xor_low) + __builtin_popcount(xor_high);
                    cost = static_cast<float>(distance);
                }

                costPtr[d] = cost;
            }
        }
    }
}

/**
 * @brief Batch Hamming distance computation with prefetching
 *
 * Optimized for cache locality and prefetching patterns.
 */
void computeHammingCostBatchNEON(const cv::Mat& censusLeft,
                                 const cv::Mat& censusRight,
                                 cv::Mat& costVolume,
                                 int minDisparity,
                                 int numDisparities) {
#ifdef __ARM_NEON
    const int width = censusLeft.cols;
    const int height = censusLeft.rows;

    const int dims[3] = {height, width, numDisparities};
    costVolume.create(3, dims, CV_32F);

    // Process multiple pixels at once for better cache usage
    const int pixelBlock = 8;

    #pragma omp parallel for schedule(dynamic)
    for (int y = 0; y < height; y++) {
        const cv::Vec2d* leftPtr = censusLeft.ptr<cv::Vec2d>(y);
        const cv::Vec2d* rightPtr = censusRight.ptr<cv::Vec2d>(y);

        for (int x = 0; x < width; x += pixelBlock) {
            int blockSize = std::min(pixelBlock, width - x);

            // Prefetch next block
            if (x + pixelBlock < width) {
                __builtin_prefetch(&leftPtr[x + pixelBlock], 0, 3);
                __builtin_prefetch(&rightPtr[x + pixelBlock], 0, 3);
            }

            for (int px = 0; px < blockSize; px++) {
                int curr_x = x + px;

                uint64_t left_low, left_high_temp;
                memcpy(&left_low, &leftPtr[curr_x][0], sizeof(uint64_t));
                memcpy(&left_high_temp, &leftPtr[curr_x][1], sizeof(uint64_t));
                uint16_t left_high = static_cast<uint16_t>(left_high_temp);

                float* costPtr = costVolume.ptr<float>(y, curr_x);

                // Unroll disparity loop for better pipelining
                for (int d = 0; d < numDisparities; d += 2) {
                    // Process two disparities at once
                    for (int di = 0; di < 2 && d + di < numDisparities; di++) {
                        int xr = curr_x - (minDisparity + d + di);
                        float cost = 80.0f;

                        if (xr >= 0 && xr < width) {
                            uint64_t right_low, right_high_temp;
                            memcpy(&right_low, &rightPtr[xr][0], sizeof(uint64_t));
                            memcpy(&right_high_temp, &rightPtr[xr][1], sizeof(uint64_t));
                            uint16_t right_high = static_cast<uint16_t>(right_high_temp);

                            cost = static_cast<float>(
                                hammingDistance80(left_low, left_high, right_low, right_high)
                            );
                        }

                        costPtr[d + di] = cost;
                    }
                }
            }
        }
    }
#else
    computeHammingCostCPU(censusLeft, censusRight, costVolume, minDisparity, numDisparities);
#endif
}

} // namespace neon
} // namespace stereo
} // namespace unlook