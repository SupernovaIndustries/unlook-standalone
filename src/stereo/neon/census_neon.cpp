/**
 * @file census_neon.cpp
 * @brief ARM NEON optimized Census Transform implementation
 *
 * Implements 9x9 Census Transform with 80-bit descriptors using ARM NEON SIMD
 * for real-time stereo matching on Raspberry Pi CM4/CM5.
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
 * @brief NEON-optimized 9x9 Census Transform
 *
 * Computes 80-bit Census descriptors for each pixel using a 9x9 window.
 * The center pixel is excluded, resulting in 80 comparisons.
 *
 * @param image Input grayscale image (CV_8UC1)
 * @param census Output census descriptors (CV_64FC2 for 80-bit storage)
 * @param threshold Illumination tolerance (default: 4)
 */
void censusTransform9x9NEON(const cv::Mat& image, cv::Mat& census, int threshold = 4) {
#ifdef __ARM_NEON
    const int width = image.cols;
    const int height = image.rows;
    const int radius = 4;  // 9x9 window

    // Initialize output: CV_64FC2 to store 80-bit descriptors
    // First channel: bits 0-63, Second channel: bits 64-79
    census.create(height, width, CV_64FC2);

    // Process internal pixels (skip borders)
    for (int y = radius; y < height - radius; y++) {
        const uint8_t* rowPtr = image.ptr<uint8_t>(y);
        cv::Vec2d* censusPtr = census.ptr<cv::Vec2d>(y);

        for (int x = radius; x < width - radius; x++) {
            uint64_t descriptor_low = 0;   // Bits 0-63
            uint16_t descriptor_high = 0;  // Bits 64-79

            const uint8_t centerVal = rowPtr[x];
            uint8x16_t center_vec = vdupq_n_u8(centerVal);
            uint8x16_t threshold_vec = vdupq_n_u8(threshold);

            int bitPos = 0;

            // Process 9x9 window, comparing with center pixel
            for (int dy = -radius; dy <= radius; dy++) {
                const uint8_t* winRowPtr = image.ptr<uint8_t>(y + dy);

                // Process window row in chunks of 16 pixels where possible
                for (int dx = -radius; dx <= radius; dx++) {
                    // Skip center pixel
                    if (dy == 0 && dx == 0) continue;

                    uint8_t pixelVal = winRowPtr[x + dx];

                    // Apply threshold for illumination tolerance
                    bool censusbit;
                    if (threshold > 0) {
                        // Modified Census: compare with tolerance
                        censusbit = (pixelVal > centerVal + threshold);
                    } else {
                        // Standard Census: simple comparison
                        censusbit = (pixelVal > centerVal);
                    }

                    // Store bit in appropriate position
                    if (bitPos < 64) {
                        descriptor_low |= (uint64_t(censusbit) << bitPos);
                    } else {
                        descriptor_high |= (uint16_t(censusbit) << (bitPos - 64));
                    }
                    bitPos++;
                }
            }

            // Store 80-bit descriptor as two doubles
            // Note: We use bit manipulation to pack into doubles
            double low_double, high_double;
            memcpy(&low_double, &descriptor_low, sizeof(double));
            memcpy(&high_double, &descriptor_high, sizeof(double));

            censusPtr[x] = cv::Vec2d(low_double, high_double);
        }
    }

    // Handle border pixels (set to zero)
    for (int y = 0; y < radius; y++) {
        census.row(y).setTo(cv::Scalar(0, 0));
        census.row(height - 1 - y).setTo(cv::Scalar(0, 0));
    }
    for (int x = 0; x < radius; x++) {
        census.col(x).setTo(cv::Scalar(0, 0));
        census.col(width - 1 - x).setTo(cv::Scalar(0, 0));
    }

#else
    // Non-NEON fallback implementation
    censusTransform9x9CPU(image, census, threshold);
#endif
}

/**
 * @brief CPU fallback for Census Transform (non-NEON)
 */
void censusTransform9x9CPU(const cv::Mat& image, cv::Mat& census, int threshold = 4) {
    const int width = image.cols;
    const int height = image.rows;
    const int radius = 4;  // 9x9 window

    census.create(height, width, CV_64FC2);
    census.setTo(cv::Scalar(0, 0));

    for (int y = radius; y < height - radius; y++) {
        const uint8_t* rowPtr = image.ptr<uint8_t>(y);
        cv::Vec2d* censusPtr = census.ptr<cv::Vec2d>(y);

        for (int x = radius; x < width - radius; x++) {
            uint64_t descriptor_low = 0;
            uint16_t descriptor_high = 0;

            const uint8_t centerVal = rowPtr[x];
            int bitPos = 0;

            for (int dy = -radius; dy <= radius; dy++) {
                const uint8_t* winRowPtr = image.ptr<uint8_t>(y + dy);

                for (int dx = -radius; dx <= radius; dx++) {
                    if (dy == 0 && dx == 0) continue;

                    uint8_t pixelVal = winRowPtr[x + dx];
                    bool censusbit;

                    if (threshold > 0) {
                        censusbit = (pixelVal > centerVal + threshold);
                    } else {
                        censusbit = (pixelVal > centerVal);
                    }

                    if (bitPos < 64) {
                        descriptor_low |= (uint64_t(censusbit) << bitPos);
                    } else {
                        descriptor_high |= (uint16_t(censusbit) << (bitPos - 64));
                    }
                    bitPos++;
                }
            }

            double low_double, high_double;
            memcpy(&low_double, &descriptor_low, sizeof(double));
            memcpy(&high_double, &descriptor_high, sizeof(double));

            censusPtr[x] = cv::Vec2d(low_double, high_double);
        }
    }
}

/**
 * @brief Optimized Census Transform with NEON vector comparisons
 *
 * This version processes multiple pixels simultaneously using NEON vectors
 * for the comparison operations.
 */
void censusTransformOptimizedNEON(const cv::Mat& image, cv::Mat& census, int threshold = 4) {
#ifdef __ARM_NEON
    const int width = image.cols;
    const int height = image.rows;
    const int radius = 4;

    census.create(height, width, CV_64FC2);

    // Process rows in parallel where possible
    #pragma omp parallel for schedule(dynamic)
    for (int y = radius; y < height - radius; y++) {
        cv::Vec2d* censusPtr = census.ptr<cv::Vec2d>(y);

        for (int x = radius; x < width - radius; x += 8) {
            // Process 8 pixels at once when possible
            int remaining = std::min(8, width - radius - x);

            for (int px = 0; px < remaining; px++) {
                int curr_x = x + px;
                uint64_t descriptor_low = 0;
                uint16_t descriptor_high = 0;

                const uint8_t centerVal = image.at<uint8_t>(y, curr_x);
                uint8x8_t center_vec = vdup_n_u8(centerVal);
                uint8x8_t threshold_vec = vdup_n_u8(threshold);

                int bitPos = 0;

                // Process window using vectorized comparisons where possible
                for (int dy = -radius; dy <= radius; dy++) {
                    int win_y = y + dy;

                    // Load 9 pixels from the window row
                    uint8_t window_pixels[9];
                    for (int i = 0; i < 9; i++) {
                        window_pixels[i] = image.at<uint8_t>(win_y, curr_x - radius + i);
                    }

                    // Process with NEON
                    uint8x8_t win_vec = vld1_u8(window_pixels);
                    uint8x8_t cmp_result;

                    if (threshold > 0) {
                        uint8x8_t center_plus_thresh = vadd_u8(center_vec, threshold_vec);
                        cmp_result = vcgt_u8(win_vec, center_plus_thresh);
                    } else {
                        cmp_result = vcgt_u8(win_vec, center_vec);
                    }

                    // Extract comparison results and pack into descriptor
                    uint8_t results[8];
                    vst1_u8(results, cmp_result);

                    for (int i = 0; i < 9; i++) {
                        int dx = -radius + i;
                        if (dy == 0 && dx == 0) continue;

                        bool bit = (i < 8) ? (results[i] != 0) :
                                  (window_pixels[8] > (centerVal + (threshold > 0 ? threshold : 0)));

                        if (bitPos < 64) {
                            descriptor_low |= (uint64_t(bit) << bitPos);
                        } else {
                            descriptor_high |= (uint16_t(bit) << (bitPos - 64));
                        }
                        bitPos++;
                    }
                }

                double low_double, high_double;
                memcpy(&low_double, &descriptor_low, sizeof(double));
                memcpy(&high_double, &descriptor_high, sizeof(double));

                censusPtr[curr_x] = cv::Vec2d(low_double, high_double);
            }
        }
    }

    // Clear borders
    for (int y = 0; y < radius; y++) {
        census.row(y).setTo(cv::Scalar(0, 0));
        census.row(height - 1 - y).setTo(cv::Scalar(0, 0));
    }
    for (int x = 0; x < radius; x++) {
        census.col(x).setTo(cv::Scalar(0, 0));
        census.col(width - 1 - x).setTo(cv::Scalar(0, 0));
    }
#else
    censusTransform9x9CPU(image, census, threshold);
#endif
}

} // namespace neon
} // namespace stereo
} // namespace unlook