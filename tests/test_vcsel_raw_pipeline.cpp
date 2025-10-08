/**
 * Test program for VCSEL pattern preservation using RAW pipeline
 *
 * This test validates that the RAW camera pipeline preserves VCSEL dot
 * patterns better than the standard preprocessing pipeline.
 */

#include <unlook/stereo/RawStereoProcessor.hpp>
#include <unlook/camera/CameraDevice.hpp>
#include <unlook/camera/CameraUtils.hpp>
#include <unlook/core/Logger.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <iomanip>
#include <chrono>
#include <filesystem>

using namespace unlook;
using namespace std;

// Test configuration
struct TestConfig {
    bool visualDisplay = false;       // Show live windows (requires display)
    bool saveImages = true;          // Save captured images
    bool runComparison = true;       // Compare RAW vs standard
    bool autoTune = false;           // Auto-tune parameters
    int numFrames = 10;              // Number of frames to capture
    string outputDir = "/tmp/vcsel_test/";
};

// Display test results
void displayResults(const stereo::RawStereoProcessor::ProcessingResult& result) {
    cout << "\n=== Processing Results ===" << endl;
    cout << "Left Variance:    " << fixed << setprecision(2) << result.leftVariance << endl;
    cout << "Right Variance:   " << fixed << setprecision(2) << result.rightVariance << endl;
    cout << "Variance Ratio:   " << fixed << setprecision(2) << result.varianceRatio << "x" << endl;
    cout << "Coverage:         " << fixed << setprecision(1) << result.coverage * 100 << "%" << endl;
    cout << "Valid Pixels:     " << result.validPixels << "/" << result.totalPixels << endl;
    cout << "Sync Error:       " << fixed << setprecision(3) << result.syncErrorMs << " ms" << endl;
    cout << "Capture Time:     " << fixed << setprecision(1) << result.captureTimeMs << " ms" << endl;
    cout << "Processing Time:  " << fixed << setprecision(1) << result.processingTimeMs << " ms" << endl;
}

// Compare pipelines
void comparePipelines(stereo::RawStereoProcessor& processor) {
    cout << "\n=== Pipeline Comparison ===" << endl;
    cout << "Comparing RAW vs Standard preprocessing pipeline..." << endl;

    stereo::RawStereoProcessor::PipelineComparison comparison;
    if (processor.comparePipelines(comparison)) {
        cout << "\n--- Variance Comparison ---" << endl;
        cout << "RAW Left:      " << fixed << setprecision(2) << comparison.rawVarianceLeft << endl;
        cout << "Standard Left: " << fixed << setprecision(2) << comparison.standardVarianceLeft << endl;
        cout << "Improvement:   +" << fixed << setprecision(1)
             << comparison.varianceImprovementLeft << "%" << endl;

        cout << "\n--- Coverage Comparison ---" << endl;
        cout << "RAW Coverage:      " << fixed << setprecision(1)
             << comparison.rawCoverage * 100 << "%" << endl;
        cout << "Standard Coverage: " << fixed << setprecision(1)
             << comparison.standardCoverage * 100 << "%" << endl;
        cout << "Improvement:       +" << fixed << setprecision(1)
             << comparison.coverageImprovement << "%" << endl;

        // Verdict
        cout << "\n--- VERDICT ---" << endl;
        if (comparison.varianceImprovementLeft > 50) {
            cout << "✓ RAW pipeline significantly preserves VCSEL patterns better!" << endl;
            cout << "  Variance improved by " << comparison.varianceImprovementLeft << "%" << endl;
        } else if (comparison.varianceImprovementLeft > 20) {
            cout << "✓ RAW pipeline moderately improves VCSEL pattern preservation" << endl;
        } else {
            cout << "⚠ Limited improvement - may need parameter tuning" << endl;
        }
    } else {
        cout << "Failed to compare pipelines" << endl;
    }
}

// Auto-tune parameters
void autoTuneParameters(stereo::RawStereoProcessor& processor) {
    cout << "\n=== Auto-tuning Parameters ===" << endl;
    cout << "Target variance: 150.0" << endl;
    cout << "Tuning SGBM parameters for optimal VCSEL pattern matching..." << endl;

    auto optimizedConfig = processor.autoTuneParameters(150.0);
    processor.setConfig(optimizedConfig);

    cout << "Optimized parameters:" << endl;
    cout << "  Block Size:       " << optimizedConfig.blockSize << endl;
    cout << "  P1:              " << optimizedConfig.p1 << endl;
    cout << "  P2:              " << optimizedConfig.p2 << endl;
    cout << "  Uniqueness Ratio: " << optimizedConfig.uniquenessRatio << endl;
}

// Save test images
void saveTestImages(const stereo::RawStereoProcessor::ProcessingResult& result,
                   const string& outputDir, int frameNum) {
    string prefix = outputDir + "frame_" + to_string(frameNum) + "_";

    // Save blue channel images
    cv::imwrite(prefix + "left_blue.png", result.leftBlue);
    cv::imwrite(prefix + "right_blue.png", result.rightBlue);

    // Save disparity (normalized for visualization)
    cv::Mat dispVis;
    cv::normalize(result.disparityMap, dispVis, 0, 255, cv::NORM_MINMAX, CV_8UC1);
    cv::imwrite(prefix + "disparity.png", dispVis);

    // Save confidence map
    cv::Mat confVis;
    cv::normalize(result.confidenceMap, confVis, 0, 255, cv::NORM_MINMAX, CV_8UC1);
    cv::imwrite(prefix + "confidence.png", confVis);

    cout << "Saved images to " << prefix << "*" << endl;
}

// Main test function
int main(int argc, char* argv[]) {
    // Initialize logging
    core::Logger::setLogLevel(core::LogLevel::INFO);
    cout << "VCSEL RAW Pipeline Test" << endl;
    cout << "========================" << endl;

    // Parse arguments
    TestConfig config;
    for (int i = 1; i < argc; i++) {
        string arg = argv[i];
        if (arg == "--display") {
            config.visualDisplay = true;
        } else if (arg == "--no-save") {
            config.saveImages = false;
        } else if (arg == "--auto-tune") {
            config.autoTune = true;
        } else if (arg == "--frames" && i + 1 < argc) {
            config.numFrames = stoi(argv[++i]);
        } else if (arg == "--help") {
            cout << "Usage: " << argv[0] << " [options]" << endl;
            cout << "Options:" << endl;
            cout << "  --display        Show live visualization" << endl;
            cout << "  --no-save       Don't save images" << endl;
            cout << "  --auto-tune     Auto-tune SGBM parameters" << endl;
            cout << "  --frames N      Number of frames to capture (default: 10)" << endl;
            return 0;
        }
    }

    // Create output directory
    if (config.saveImages) {
        filesystem::create_directories(config.outputDir);
        cout << "Output directory: " << config.outputDir << endl;
    }

    // Create RAW stereo processor
    stereo::RawStereoProcessor::Config processorConfig;
    processorConfig.useRawPipeline = true;
    processorConfig.extractBlueOnly = true;
    processorConfig.computeVariance = true;
    processorConfig.saveRawImages = config.saveImages;
    processorConfig.saveVarianceMaps = config.saveImages;
    processorConfig.debugPath = config.outputDir;

    // SGBM parameters optimized for VCSEL
    processorConfig.blockSize = 5;        // Small for dots
    processorConfig.numDisparities = 128;
    processorConfig.p1 = 50;
    processorConfig.p2 = 200;
    processorConfig.uniquenessRatio = 10;
    processorConfig.speckleWindowSize = 100;
    processorConfig.speckleRange = 32;

    stereo::RawStereoProcessor processor(processorConfig);
    processor.setDebugMode(true);

    // Initialize processor
    cout << "\nInitializing RAW stereo processor..." << endl;
    if (!processor.initialize()) {
        cerr << "Failed to initialize processor: " << processor.getLastError() << endl;
        return 1;
    }
    cout << "✓ Processor initialized" << endl;

    // Auto-tune if requested
    if (config.autoTune) {
        autoTuneParameters(processor);
    }

    // Process frames
    cout << "\nCapturing and processing " << config.numFrames << " frames..." << endl;

    double totalVarianceLeft = 0;
    double totalVarianceRight = 0;
    double totalCoverage = 0;
    double maxVariance = 0;
    double minVariance = 1000;

    for (int i = 0; i < config.numFrames; i++) {
        cout << "\n--- Frame " << (i + 1) << "/" << config.numFrames << " ---" << endl;

        stereo::RawStereoProcessor::ProcessingResult result;
        if (!processor.processStereo(result)) {
            cerr << "Failed to process frame: " << processor.getLastError() << endl;
            continue;
        }

        // Display results
        displayResults(result);

        // Accumulate statistics
        totalVarianceLeft += result.leftVariance;
        totalVarianceRight += result.rightVariance;
        totalCoverage += result.coverage;
        maxVariance = max(maxVariance, max(result.leftVariance, result.rightVariance));
        minVariance = min(minVariance, min(result.leftVariance, result.rightVariance));

        // Save images
        if (config.saveImages) {
            saveTestImages(result, config.outputDir, i);
        }

        // Display if requested
        if (config.visualDisplay) {
            cv::imshow("Left Blue", result.leftBlue);
            cv::imshow("Right Blue", result.rightBlue);

            cv::Mat dispVis;
            cv::normalize(result.disparityMap, dispVis, 0, 255, cv::NORM_MINMAX, CV_8UC1);
            cv::applyColorMap(dispVis, dispVis, cv::COLORMAP_JET);
            cv::imshow("Disparity", dispVis);

            if (cv::waitKey(100) == 27) break;  // ESC to exit
        }
    }

    // Summary statistics
    cout << "\n=== Summary Statistics ===" << endl;
    cout << "Average Left Variance:  " << fixed << setprecision(2)
         << totalVarianceLeft / config.numFrames << endl;
    cout << "Average Right Variance: " << fixed << setprecision(2)
         << totalVarianceRight / config.numFrames << endl;
    cout << "Average Coverage:       " << fixed << setprecision(1)
         << (totalCoverage / config.numFrames) * 100 << "%" << endl;
    cout << "Variance Range:         " << fixed << setprecision(2)
         << minVariance << " - " << maxVariance << endl;

    // Compare pipelines if requested
    if (config.runComparison) {
        comparePipelines(processor);
    }

    // Final assessment
    cout << "\n=== VCSEL Pattern Preservation Assessment ===" << endl;
    double avgVariance = (totalVarianceLeft + totalVarianceRight) / (2.0 * config.numFrames);

    if (avgVariance > 150) {
        cout << "✓ EXCELLENT: VCSEL patterns are well preserved (variance: "
             << avgVariance << ")" << endl;
        cout << "  The RAW pipeline successfully maintains dot pattern detail" << endl;
    } else if (avgVariance > 100) {
        cout << "✓ GOOD: VCSEL patterns are adequately preserved (variance: "
             << avgVariance << ")" << endl;
        cout << "  The RAW pipeline shows improvement over standard processing" << endl;
    } else if (avgVariance > 86) {
        cout << "⚠ MODERATE: Some improvement over standard (variance: "
             << avgVariance << ")" << endl;
        cout << "  Consider adjusting exposure or VCSEL power" << endl;
    } else {
        cout << "✗ POOR: VCSEL patterns are not well preserved (variance: "
             << avgVariance << ")" << endl;
        cout << "  Check hardware setup and lighting conditions" << endl;
    }

    // Recommendations
    cout << "\n=== Recommendations ===" << endl;
    if (avgVariance < 100) {
        cout << "• Increase VCSEL power or exposure time" << endl;
        cout << "• Ensure cameras are properly focused" << endl;
        cout << "• Check for ambient IR interference" << endl;
    }
    if (totalCoverage / config.numFrames < 0.7) {
        cout << "• Coverage is low - adjust disparity range" << endl;
        cout << "• Consider reducing uniqueness ratio" << endl;
    }

    cout << "\nTest completed successfully!" << endl;

    return 0;
}