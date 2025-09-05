#include <opencv2/imgproc.hpp>

namespace unlook {
namespace utils {

class ImageUtils {
public:
    static void convertBayerToRGB(const cv::Mat& bayer, cv::Mat& rgb) {
        cv::cvtColor(bayer, rgb, cv::COLOR_BayerBG2RGB);
    }
    
    static void normalizeImage(const cv::Mat& input, cv::Mat& output) {
        cv::normalize(input, output, 0, 255, cv::NORM_MINMAX, CV_8U);
    }
    
    static double computeSharpness(const cv::Mat& image) {
        cv::Mat laplacian;
        cv::Laplacian(image, laplacian, CV_64F);
        cv::Scalar mean, stddev;
        cv::meanStdDev(laplacian, mean, stddev);
        return stddev.val[0] * stddev.val[0];
    }
};

} // namespace utils
} // namespace unlook