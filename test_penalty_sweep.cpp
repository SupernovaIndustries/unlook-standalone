// Sweep through penalty values to find optimal
#include "../unlook-standalone/test_offline_scan/reference_sgm_census.hpp"
#include <opencv2/opencv.hpp>
#include <numeric>

int main() {
    cv::Mat left = cv::imread("/tmp/left_rect.png", cv::IMREAD_GRAYSCALE);
    cv::Mat right = cv::imread("/tmp/right_rect.png", cv::IMREAD_GRAYSCALE);

    std::cout << "PENALTY SWEEP (numDisparities=384, Census 5x5, RAW hamming):" << std::endl;
    std::cout << "Expected disparity: ~299 px" << std::endl;
    std::cout << std::endl;

    // Test different penalty combinations
    int P1_values[] = {3, 10, 20, 30, 50};
    int P2_values[] = {20, 40, 80, 120, 200};

    cv::Rect roi(550, 300, 150, 100);

    for (int P1 : P1_values) {
        for (int P2 : P2_values) {
            if (P2 <= P1) continue;  // P2 must be > P1

            unlook::stereo::reference::ReferenceSGMCensus::Config config;
            config.censusWindowSize = 5;
            config.numDisparities = 384;
            config.P1 = P1;
            config.P2 = P2;
            config.use8Paths = true;
            config.verbose = false;

            unlook::stereo::reference::ReferenceSGMCensus sgm(config);
            auto result = sgm.compute(left, right);

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
                float median = roiDisps[roiDisps.size() / 2];
                float accuracy = (median / 299.0 * 100.0);

                std::cout << "P1=" << P1 << ", P2=" << P2
                          << ": median=" << median << " px"
                          << ", accuracy=" << accuracy << "%"
                          << ", validPixels=" << roiDisps.size() << "/" << (roi.width * roi.height)
                          << std::endl;
            }
        }
    }

    return 0;
}
