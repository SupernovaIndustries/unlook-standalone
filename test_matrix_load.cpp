#include <opencv2/opencv.hpp>
#include <iostream>

int main() {
    cv::FileStorage fs("/home/alessandro/unlook-standalone/calibration/calib_boofcv_test3.yaml", cv::FileStorage::READ);
    if (!fs.isOpened()) {
        std::cerr << "Failed to open file" << std::endl;
        return 1;
    }
    
    cv::Mat R1, R2, P1, P2;
    fs["R1"] >> R1;
    fs["R2"] >> R2;
    fs["P1"] >> P1;
    fs["P2"] >> P2;
    
    std::cout << "R1 empty: " << R1.empty() << ", size: " << R1.size() << std::endl;
    std::cout << "R2 empty: " << R2.empty() << ", size: " << R2.size() << std::endl;
    std::cout << "P1 empty: " << P1.empty() << ", size: " << P1.size() << std::endl;
    std::cout << "P2 empty: " << P2.empty() << ", size: " << P2.size() << std::endl;
    
    if (!R1.empty()) {
        std::cout << "R1[0,0]: " << R1.at<double>(0,0) << std::endl;
    }
    
    fs.release();
    return 0;
}
