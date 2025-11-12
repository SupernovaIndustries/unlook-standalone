// Test with BEST configuration from sweep: P1=20, P2=40, numDisp=384
#include "../unlook-standalone/test_offline_scan/reference_sgm_census.hpp"
#include <opencv2/opencv.hpp>
#include <numeric>

int main() {
    cv::Mat left = cv::imread("/tmp/left_rect.png", cv::IMREAD_GRAYSCALE);
    cv::Mat right = cv::imread("/tmp/right_rect.png", cv::IMREAD_GRAYSCALE);

    std::cout << "BEST CONFIGURATION (from penalty sweep):" << std::endl;
    std::cout << "  Census: 5x5 window" << std::endl;
    std::cout << "  Cost: RAW hamming distance (no *10 scaling)" << std::endl;
    std::cout << "  numDisparities: 384 (was 256 - too small!)" << std::endl;
    std::cout << "  P1: 20 (was 300 - too large!)" << std::endl;
    std::cout << "  P2: 40 (was 1200 - WAY too large!)" << std::endl;
    std::cout << std::endl;

    unlook::stereo::reference::ReferenceSGMCensus::Config config;
    config.censusWindowSize = 5;
    config.numDisparities = 384;
    config.P1 = 20;
    config.P2 = 40;
    config.use8Paths = true;
    config.verbose = true;

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

        std::cout << "\nROI ANALYSIS (hand region):" << std::endl;
        std::cout << "  Mean disparity: " << roiMean << " px" << std::endl;
        std::cout << "  Median disparity: " << roiMedian << " px" << std::endl;
        std::cout << "  Max disparity: " << roiMax << " px" << std::endl;
        std::cout << "  Expected: ~299 px" << std::endl;
        std::cout << "  Accuracy: " << (roiMedian / 299.0 * 100.0) << "%" << std::endl;

        // Calculate expected depth
        float focal = 1075.7954;  // From calibration
        float baseline = 69.39;    // mm
        float depthMean = (focal * baseline) / roiMean;
        float depthMedian = (focal * baseline) / roiMedian;

        std::cout << "\nDEPTH ANALYSIS:" << std::endl;
        std::cout << "  Mean depth: " << depthMean << " mm (" << depthMean/10 << " cm)" << std::endl;
        std::cout << "  Median depth: " << depthMedian << " mm (" << depthMedian/10 << " cm)" << std::endl;
        std::cout << "  Expected: ~450 mm (45 cm)" << std::endl;
    }

    // Save visualization
    cv::Mat disp8;
    result.disparity.convertTo(disp8, CV_8U, 255.0 / (384 * 16.0));
    cv::Mat disp_color;
    cv::applyColorMap(disp8, disp_color, cv::COLORMAP_JET);
    cv::rectangle(disp_color, roi, cv::Scalar(0, 255, 0), 2);
    cv::imwrite("/tmp/disparity_BEST.png", disp_color);
    std::cout << "\nSaved visualization to /tmp/disparity_BEST.png" << std::endl;

    return 0;
}
