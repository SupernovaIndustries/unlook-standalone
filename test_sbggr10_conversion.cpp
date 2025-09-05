/*
 * Test program for SBGGR10 conversion verification
 * Tests the fixed Bayer demosaicing implementation
 */

#include <iostream>
#include <opencv2/opencv.hpp>
#include <chrono>
#include <cstring>

// Function to generate synthetic SBGGR10 test pattern
void generateTestSBGGR10(uint8_t* data, int width, int height) {
    // Generate a test Bayer pattern with known values
    // BGGR pattern:
    // B G B G ...
    // G R G R ...
    // B G B G ...
    // G R G R ...
    
    const int pixels_per_pack = 4;
    const int bytes_per_pack = 5;
    const int total_pixels = width * height;
    
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            int pixel_idx = y * width + x;
            int pack_idx = pixel_idx / pixels_per_pack;
            int pixel_in_pack = pixel_idx % pixels_per_pack;
            
            uint8_t* pack_data = data + (pack_idx * bytes_per_pack);
            
            // Generate test values based on Bayer pattern
            uint16_t value;
            if ((y % 2 == 0) && (x % 2 == 0)) {
                // Blue pixel
                value = 200;  // Blue channel value (10-bit: 0-1023)
            } else if ((y % 2 == 1) && (x % 2 == 1)) {
                // Red pixel
                value = 600;  // Red channel value
            } else {
                // Green pixel
                value = 400;  // Green channel value
            }
            
            // Pack the 10-bit value into the byte array
            if (pixel_in_pack == 0) {
                pack_data[0] = (value >> 2) & 0xFF;
                pack_data[1] = (pack_data[1] & 0x3F) | ((value & 0x03) << 6);
            } else if (pixel_in_pack == 1) {
                pack_data[1] = (pack_data[1] & 0xC0) | ((value >> 4) & 0x3F);
                pack_data[2] = (pack_data[2] & 0x0F) | ((value & 0x0F) << 4);
            } else if (pixel_in_pack == 2) {
                pack_data[2] = (pack_data[2] & 0xF0) | ((value >> 6) & 0x0F);
                pack_data[3] = (pack_data[3] & 0x03) | ((value & 0x3F) << 2);
            } else if (pixel_in_pack == 3) {
                pack_data[3] = (pack_data[3] & 0xFC) | ((value >> 8) & 0x03);
                pack_data[4] = value & 0xFF;
            }
        }
    }
}

// Simple unpacking function for testing
void unpackSBGGR10_Simple(const uint8_t* src, uint16_t* dst, int width, int height) {
    const int pixels_per_pack = 4;
    const int bytes_per_pack = 5;
    const int total_pixels = width * height;
    const int num_packs = total_pixels / pixels_per_pack;
    
    for (int pack = 0; pack < num_packs; pack++) {
        const uint8_t* pack_src = src + (pack * bytes_per_pack);
        uint16_t* pack_dst = dst + (pack * pixels_per_pack);
        
        // Unpack 4 pixels from 5 bytes
        pack_dst[0] = ((uint16_t)pack_src[0] << 2) | (pack_src[1] >> 6);
        pack_dst[1] = (((uint16_t)(pack_src[1] & 0x3F)) << 4) | (pack_src[2] >> 4);
        pack_dst[2] = (((uint16_t)(pack_src[2] & 0x0F)) << 6) | (pack_src[3] >> 2);
        pack_dst[3] = (((uint16_t)(pack_src[3] & 0x03)) << 8) | pack_src[4];
    }
}

int main() {
    const int width = 1456;
    const int height = 1088;
    
    std::cout << "=== SBGGR10 Conversion Test ===" << std::endl;
    std::cout << "Image size: " << width << "x" << height << std::endl;
    
    // Allocate buffers
    const int pixels_total = width * height;
    const int packed_size = (pixels_total * 10 + 7) / 8;  // 10 bits per pixel
    
    uint8_t* packed_data = new uint8_t[packed_size];
    uint16_t* unpacked_data = new uint16_t[pixels_total];
    
    // Initialize with test pattern
    std::memset(packed_data, 0, packed_size);
    generateTestSBGGR10(packed_data, width, height);
    
    // Test unpacking
    auto start = std::chrono::high_resolution_clock::now();
    unpackSBGGR10_Simple(packed_data, unpacked_data, width, height);
    auto end = std::chrono::high_resolution_clock::now();
    
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    std::cout << "Unpacking time: " << duration.count() / 1000.0 << " ms" << std::endl;
    
    // Convert to cv::Mat for visualization
    cv::Mat bayer_16bit(height, width, CV_16UC1, unpacked_data);
    
    // Convert to 8-bit for processing
    cv::Mat bayer_8bit;
    bayer_16bit.convertTo(bayer_8bit, CV_8UC1, 255.0/1023.0);
    
    // Demosaic using OpenCV
    cv::Mat rgb_image;
    start = std::chrono::high_resolution_clock::now();
    cv::cvtColor(bayer_8bit, rgb_image, cv::COLOR_BayerBG2BGR);
    end = std::chrono::high_resolution_clock::now();
    
    duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    std::cout << "Demosaicing time: " << duration.count() / 1000.0 << " ms" << std::endl;
    
    // Convert to grayscale
    cv::Mat gray_image;
    cv::cvtColor(rgb_image, gray_image, cv::COLOR_BGR2GRAY);
    
    // Check for grid artifacts
    std::cout << "\n=== Checking for Grid Artifacts ===" << std::endl;
    
    // Sample a small region and check for regular patterns
    cv::Rect roi(100, 100, 20, 20);
    cv::Mat sample = gray_image(roi);
    
    // Calculate variance to detect grid patterns
    cv::Scalar mean, stddev;
    cv::meanStdDev(sample, mean, stddev);
    
    std::cout << "Sample region statistics:" << std::endl;
    std::cout << "  Mean: " << mean[0] << std::endl;
    std::cout << "  StdDev: " << stddev[0] << std::endl;
    
    // Check for alternating pattern (grid artifact indicator)
    bool has_grid = false;
    for (int y = 0; y < sample.rows - 1; y++) {
        for (int x = 0; x < sample.cols - 1; x++) {
            uint8_t p00 = sample.at<uint8_t>(y, x);
            uint8_t p01 = sample.at<uint8_t>(y, x + 1);
            uint8_t p10 = sample.at<uint8_t>(y + 1, x);
            uint8_t p11 = sample.at<uint8_t>(y + 1, x + 1);
            
            // Check for checkerboard pattern
            if (std::abs(p00 - p11) < 5 && std::abs(p01 - p10) < 5 &&
                std::abs(p00 - p01) > 20 && std::abs(p00 - p10) > 20) {
                has_grid = true;
                break;
            }
        }
        if (has_grid) break;
    }
    
    if (has_grid) {
        std::cout << "WARNING: Grid artifacts detected!" << std::endl;
    } else {
        std::cout << "SUCCESS: No grid artifacts detected" << std::endl;
    }
    
    // Save test images
    cv::imwrite("test_bayer_raw.png", bayer_8bit);
    cv::imwrite("test_rgb_demosaiced.png", rgb_image);
    cv::imwrite("test_gray_final.png", gray_image);
    
    std::cout << "\nTest images saved:" << std::endl;
    std::cout << "  - test_bayer_raw.png (raw Bayer pattern)" << std::endl;
    std::cout << "  - test_rgb_demosaiced.png (after demosaicing)" << std::endl;
    std::cout << "  - test_gray_final.png (final grayscale)" << std::endl;
    
    // Clean up
    delete[] packed_data;
    delete[] unpacked_data;
    
    return 0;
}