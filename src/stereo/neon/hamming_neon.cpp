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
 * @brief Compute Hamming distance between two 24-bit descriptors (5x5 Census)
 * OPTIMIZED FOR VCSEL DOT PATTERN
 */
inline int hammingDistance24(uint32_t desc1, uint32_t desc2) {
    // XOR to find differing bits, then count (only 24 bits used)
    uint32_t xor_val = desc1 ^ desc2;

#ifdef __ARM_NEON
    // Use NEON POPCOUNT for hardware acceleration
    uint8x8_t bytes = vreinterpret_u8_u32(vdup_n_u32(xor_val));
    uint8x8_t popcount = vcnt_u8(bytes);
    return vaddlv_u8(popcount);
#else
    // CPU fallback
    return __builtin_popcount(xor_val);
#endif
}

/**
 * @brief NEON-optimized Hamming cost volume computation (24-bit Census)
 *
 * Computes matching costs between left and right 24-bit Census descriptors
 * using NEON POPCOUNT instructions for maximum performance.
 *
 * OPTIMIZED FOR: VCSEL dot pattern stereo matching
 *
 * @param censusLeft Left census descriptors (CV_32FC1)
 * @param censusRight Right census descriptors (CV_32FC1)
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
        const float* leftPtr = censusLeft.ptr<float>(y);
        const float* rightPtr = censusRight.ptr<float>(y);

        for (int x = 0; x < width; x++) {
            // Extract left descriptor (24-bit stored as float)
            uint32_t left_desc;
            memcpy(&left_desc, &leftPtr[x], sizeof(uint32_t));

            // Compute costs for all disparities
            for (int d = 0; d < numDisparities; d++) {
                int xr = x - (minDisparity + d);
                float cost = 24.0f;  // Maximum cost (all 24 bits different)

                if (xr >= 0 && xr < width) {
                    // Extract right descriptor
                    uint32_t right_desc;
                    memcpy(&right_desc, &rightPtr[xr], sizeof(uint32_t));

                    // Compute Hamming distance using NEON
                    cost = static_cast<float>(
                        hammingDistance24(left_desc, right_desc)
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
        const float* leftPtr = censusLeft.ptr<float>(y);
        const float* rightPtr = censusRight.ptr<float>(y);

        for (int x = 0; x < width; x++) {
            uint32_t left_desc;
            memcpy(&left_desc, &leftPtr[x], sizeof(uint32_t));

            float* costPtr = costVolume.ptr<float>(y, x);

            // Process disparities in blocks of 4
            for (int d = 0; d < numDisparities; d += dispBlock) {
                float cost_array[4] = {24.0f, 24.0f, 24.0f, 24.0f};
                int remaining = std::min(dispBlock, numDisparities - d);

                for (int i = 0; i < remaining; i++) {
                    int xr = x - (minDisparity + d + i);

                    if (xr >= 0 && xr < width) {
                        uint32_t right_desc;
                        memcpy(&right_desc, &rightPtr[xr], sizeof(uint32_t));

                        cost_array[i] = static_cast<float>(
                            hammingDistance24(left_desc, right_desc)
                        );
                    }
                }

                // Store costs using NEON store
                float32x4_t costs = vld1q_f32(cost_array);
                vst1q_f32(&costPtr[d], costs);
            }
        }
    }
#else
    computeHammingCostCPU(censusLeft, censusRight, costVolume, minDisparity, numDisparities);
#endif
}

/**
 * @brief CPU fallback for Hamming cost computation (24-bit)
 * OPTIMIZED FOR VCSEL DOT PATTERN
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
        const float* leftPtr = censusLeft.ptr<float>(y);
        const float* rightPtr = censusRight.ptr<float>(y);

        for (int x = 0; x < width; x++) {
            uint32_t left_desc;
            memcpy(&left_desc, &leftPtr[x], sizeof(uint32_t));

            float* costPtr = costVolume.ptr<float>(y, x);

            for (int d = 0; d < numDisparities; d++) {
                int xr = x - (minDisparity + d);
                float cost = 24.0f;  // Max cost for 24-bit

                if (xr >= 0 && xr < width) {
                    uint32_t right_desc;
                    memcpy(&right_desc, &rightPtr[xr], sizeof(uint32_t));

                    cost = static_cast<float>(
                        hammingDistance24(left_desc, right_desc)
                    );
                }

                costPtr[d] = cost;
            }
        }
    }
}

// Note: computeHammingCostBatchNEON removed - was for 80-bit Census, not needed for 5x5 (24-bit)

} // namespace neon
} // namespace stereo
} // namespace unlook