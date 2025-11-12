// Test with NEW DEFAULTS (no manual config override)
#include "../unlook-standalone/test_offline_scan/reference_sgm_census.hpp"
#include <opencv2/opencv.hpp>
#include <numeric>

int main() {
    cv::Mat left = cv::imread("/tmp/left_rect.png", cv::IMREAD_GRAYSCALE);
    cv::Mat right = cv::imread("/tmp/right_rect.png", cv::IMREAD_GRAYSCALE);

    std::cout << "=== TEST WITH NEW DEFAULT PARAMETERS ===" << std::endl;
    std::cout << "Using default Config() without manual overrides" << std::endl;
    std::cout << std::endl;

    // Create config with NEW DEFAULTS (not manually setting anything!)
    unlook::stereo::reference::ReferenceSGMCensus::Config config;
    // NO manual config.P1 = 20, config.P2 = 40, etc.
    // Just use the defaults from the header!

    std::cout << "Default config values:" << std::endl;
    std::cout << "  censusWindowSize: " << config.censusWindowSize << std::endl;
    std::cout << "  numDisparities: " << config.numDisparities << std::endl;
    std::cout << "  P1: " << config.P1 << std::endl;
    std::cout << "  P2: " << config.P2 << std::endl;
    std::cout << "  use8Paths: " << (config.use8Paths ? "true" : "false") << std::endl;
    std::cout << std::endl;

    unlook::stereo::reference::ReferenceSGMCensus sgm(config);
    auto result = sgm.compute(left, right);

    // Analyze ROI
    cv::Rect roi(550, 300, 150, 100);
    cv::Mat roiDisp = result.disparity(roi);
    std::vector<float> roiDisps;
    for (int y = 0; y < roiDisp.rows; y++) {
        const int16_t* row = roiDisp.ptr<int16_t>(y);
        for (int x = 0; x < roiDisp.cols; x++) {
            if (row[x] > 0) roiDisps.push_back(row[x] / 16.0f);
        }
    }

    if (!roiDisps.empty()) {
        std::sort(roiDisps.begin(), roiDisps.end());
        float roiMean = std::accumulate(roiDisps.begin(), roiDisps.end(), 0.0f) / roiDisps.size();
        float roiMedian = roiDisps[roiDisps.size() / 2];
        float roiMax = roiDisps.back();

        std::cout << "\n=== RESULTS ===" << std::endl;
        std::cout << "ROI disparity:" << std::endl;
        std::cout << "  Mean: " << roiMean << " px" << std::endl;
        std::cout << "  Median: " << roiMedian << " px" << std::endl;
        std::cout << "  Max: " << roiMax << " px" << std::endl;
        std::cout << "  Expected: ~299 px" << std::endl;
        std::cout << "  Accuracy: " << (roiMedian / 299.0 * 100.0) << "%" << std::endl;

        // Calculate depth
        float focal = 1075.7954;
        float baseline = 69.39;
        float depthMedian = (focal * baseline) / roiMedian;

        std::cout << "\nDepth:" << std::endl;
        std::cout << "  Median: " << depthMedian << " mm (" << depthMedian/10 << " cm)" << std::endl;
        std::cout << "  Expected: ~450 mm (45 cm)" << std::endl;

        // Verdict
        std::cout << "\n=== VERDICT ===" << std::endl;
        if (roiMedian >= 235 && roiMedian <= 245) {
            std::cout << "✅ EXCELLENT! Default parameters are working correctly (80%+ accuracy)" << std::endl;
        } else if (roiMedian >= 200) {
            std::cout << "⚠️  GOOD but could be better (" << (roiMedian / 299.0 * 100.0) << "% accuracy)" << std::endl;
        } else {
            std::cout << "❌ POOR - defaults not working as expected!" << std::endl;
        }
    }

    return 0;
}
