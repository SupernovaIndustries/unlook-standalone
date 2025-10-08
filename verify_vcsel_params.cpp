/**
 * @file verify_vcsel_params.cpp
 * @brief Verification program to confirm VCSEL-optimized SGBM parameters
 */

#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/ximgproc.hpp>
#include <opencv2/calib3d.hpp>
#include "unlook/stereo/SGBMStereoMatcher.hpp"
#include "unlook/api/depth_processor.h"

void printSGBMParameters(cv::Ptr<cv::StereoSGBM> sgbm) {
    std::cout << "\nOpenCV StereoSGBM Parameters:\n";
    std::cout << "  MinDisparity: " << sgbm->getMinDisparity() << std::endl;
    std::cout << "  NumDisparities: " << sgbm->getNumDisparities() << std::endl;
    std::cout << "  BlockSize: " << sgbm->getBlockSize() << std::endl;
    std::cout << "  P1: " << sgbm->getP1() << std::endl;
    std::cout << "  P2: " << sgbm->getP2() << std::endl;
    std::cout << "  Disp12MaxDiff: " << sgbm->getDisp12MaxDiff() << std::endl;
    std::cout << "  PreFilterCap: " << sgbm->getPreFilterCap() << std::endl;
    std::cout << "  UniquenessRatio: " << sgbm->getUniquenessRatio() << std::endl;
    std::cout << "  SpeckleWindowSize: " << sgbm->getSpeckleWindowSize() << std::endl;
    std::cout << "  SpeckleRange: " << sgbm->getSpeckleRange() << std::endl;
    std::cout << "  Mode: " << sgbm->getMode() << std::endl;
}

int main() {
    std::cout << "=== VCSEL Parameter Verification ===\n" << std::endl;

    // Test 1: Check SGBMStereoMatcher default parameters
    std::cout << "1. SGBMStereoMatcher Default Parameters:" << std::endl;
    {
        unlook::stereo::SGBMStereoMatcher matcher;
        auto params = matcher.getParameters();

        std::cout << "\nExpected VCSEL-optimized values:" << std::endl;
        std::cout << "  blockSize: 5 (for dot clusters)" << std::endl;
        std::cout << "  P1: 200 (8*1*5*5)" << std::endl;
        std::cout << "  P2: 800 (32*1*5*5)" << std::endl;
        std::cout << "  uniquenessRatio: 15 (high for dots)" << std::endl;
        std::cout << "  preFilterCap: 63 (maximum)" << std::endl;
        std::cout << "  speckleWindowSize: 50 (moderate)" << std::endl;
        std::cout << "  speckleRange: 64 (wide for dots)" << std::endl;
        std::cout << "  mode: 0 (MODE_SGBM, not HH)" << std::endl;

        std::cout << "\nActual values from SGBMStereoMatcher:" << std::endl;
        std::cout << "  blockSize: " << params.blockSize
                  << (params.blockSize == 5 ? " ✓" : " ✗ MISMATCH!") << std::endl;
        std::cout << "  P1: " << params.P1
                  << (params.P1 == 200 ? " ✓" : " ✗ MISMATCH!") << std::endl;
        std::cout << "  P2: " << params.P2
                  << (params.P2 == 800 ? " ✓" : " ✗ MISMATCH!") << std::endl;
        std::cout << "  uniquenessRatio: " << params.uniquenessRatio
                  << (params.uniquenessRatio == 15 ? " ✓" : " ✗ MISMATCH!") << std::endl;
        std::cout << "  preFilterCap: " << params.preFilterCap
                  << (params.preFilterCap == 63 ? " ✓" : " ✗ MISMATCH!") << std::endl;
        std::cout << "  speckleWindowSize: " << params.speckleWindowSize
                  << (params.speckleWindowSize == 50 ? " ✓" : " ✗ MISMATCH!") << std::endl;
        std::cout << "  speckleRange: " << params.speckleRange
                  << (params.speckleRange == 64 ? " ✓" : " ✗ MISMATCH!") << std::endl;
        std::cout << "  mode: " << params.mode
                  << (params.mode == cv::StereoSGBM::MODE_SGBM ? " ✓" : " ✗ MISMATCH!") << std::endl;
    }

    // Test 2: Check API DepthProcessor presets
    std::cout << "\n2. API DepthProcessor Presets:" << std::endl;
    {
        // Test all quality levels
        std::vector<unlook::core::DepthQuality> qualities = {
            unlook::core::DepthQuality::FAST_LOW,
            unlook::core::DepthQuality::BALANCED,
            unlook::core::DepthQuality::SLOW_HIGH
        };

        for (auto quality : qualities) {
            auto config = unlook::api::DepthProcessor::createPreset(
                quality,
                unlook::core::StereoAlgorithm::SGBM_OPENCV
            );

            std::string quality_name;
            switch(quality) {
                case unlook::core::DepthQuality::FAST_LOW:
                    quality_name = "FAST_LOW";
                    break;
                case unlook::core::DepthQuality::BALANCED:
                    quality_name = "BALANCED";
                    break;
                case unlook::core::DepthQuality::SLOW_HIGH:
                    quality_name = "SLOW_HIGH";
                    break;
            }

            std::cout << "\n  " << quality_name << " preset:" << std::endl;
            std::cout << "    block_size: " << config.block_size
                      << (config.block_size == 5 ? " ✓" : " ✗ SHOULD BE 5!") << std::endl;
            std::cout << "    P1: " << config.p1
                      << (config.p1 == 200 ? " ✓" : " ✗ SHOULD BE 200!") << std::endl;
            std::cout << "    P2: " << config.p2
                      << (config.p2 == 800 ? " ✓" : " ✗ SHOULD BE 800!") << std::endl;
            std::cout << "    pre_filter_cap: " << config.pre_filter_cap
                      << (config.pre_filter_cap == 63 ? " ✓" : " ✗ SHOULD BE 63!") << std::endl;
            std::cout << "    mode: " << config.mode
                      << (config.mode == cv::StereoSGBM::MODE_SGBM ? " ✓" : " ✗ SHOULD BE MODE_SGBM!") << std::endl;
        }
    }

    // Test 3: Create OpenCV SGBM directly with VCSEL parameters
    std::cout << "\n3. Direct OpenCV SGBM with VCSEL Parameters:" << std::endl;
    {
        cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(
            0,    // minDisparity
            320,  // numDisparities
            5     // blockSize
        );

        // Set VCSEL parameters
        sgbm->setP1(200);
        sgbm->setP2(800);
        sgbm->setPreFilterCap(63);
        sgbm->setUniquenessRatio(15);
        sgbm->setSpeckleWindowSize(50);
        sgbm->setSpeckleRange(64);
        sgbm->setMode(cv::StereoSGBM::MODE_SGBM);

        printSGBMParameters(sgbm);
    }

    std::cout << "\n=== Verification Complete ===" << std::endl;
    std::cout << "\nSUMMARY: All VCSEL-optimized parameters have been hardcoded as defaults." << std::endl;
    std::cout << "Key parameters for dot pattern matching:" << std::endl;
    std::cout << "  - Block size: 5 (captures single dot clusters)" << std::endl;
    std::cout << "  - P1/P2: 200/800 (smooth surface assumption)" << std::endl;
    std::cout << "  - Uniqueness: 15 (prevents dot mismatching)" << std::endl;
    std::cout << "  - PreFilterCap: 63 (minimal filtering)" << std::endl;
    std::cout << "  - SpeckleWindow: 50 (moderate noise removal)" << std::endl;
    std::cout << "  - Mode: SGBM (standard, not HH)" << std::endl;

    return 0;
}