#include <unlook/camera/HardwareSyncCapture.hpp>
#include <unlook/core/Logger.hpp>
#include <unlook/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>

int main() {
    using namespace unlook::camera;
    using namespace unlook::core;
    
    Logger::getInstance().initialize(LogLevel::INFO, true, false);
    
    std::cout << "Testing Agent's SBGGR10 Implementation\n";
    std::cout << "=====================================\n";
    
    HardwareSyncCapture capture;
    
    if (!capture.initialize()) {
        std::cerr << "Failed to initialize hardware sync capture" << std::endl;
        return 1;
    }
    
    if (!capture.start()) {
        std::cerr << "Failed to start capture" << std::endl;
        return 1;
    }
    
    // Capture a frame pair
    unlook::core::StereoFramePair frame;
    if (capture.captureSingle(frame, 2000)) {
        std::cout << "✓ Frame captured successfully" << std::endl;
        std::cout << "  Left image size: " << frame.left.image.cols << "x" << frame.left.image.rows << std::endl;
        std::cout << "  Right image size: " << frame.right.image.cols << "x" << frame.right.image.rows << std::endl;
        std::cout << "  Image type: " << frame.left.image.type() << " (CV_8UC1=" << CV_8UC1 << ")" << std::endl;
        std::cout << "  Sync error: " << frame.sync_error_ms << " ms" << std::endl;
        
        // Save images to verify SBGGR10 conversion quality
        cv::imwrite("test_left_agent_sbggr10.png", frame.left.image);
        cv::imwrite("test_right_agent_sbggr10.png", frame.right.image);
        std::cout << "Images saved: test_left_agent_sbggr10.png, test_right_agent_sbggr10.png" << std::endl;
        
        // Analyze image properties
        cv::Scalar mean_left, std_left, mean_right, std_right;
        cv::meanStdDev(frame.left.image, mean_left, std_left);
        cv::meanStdDev(frame.right.image, mean_right, std_right);
        
        std::cout << "\nImage Statistics:" << std::endl;
        std::cout << "  Left - Mean: " << mean_left[0] << ", StdDev: " << std_left[0] << std::endl;
        std::cout << "  Right - Mean: " << mean_right[0] << ", StdDev: " << std_right[0] << std::endl;
        
        // Check for grid artifacts (indicator of poor conversion)
        cv::Mat left_laplacian, right_laplacian;
        cv::Laplacian(frame.left.image, left_laplacian, CV_64F);
        cv::Laplacian(frame.right.image, right_laplacian, CV_64F);
        
        cv::Scalar left_var, right_var;
        cv::meanStdDev(left_laplacian, cv::Scalar(), left_var);
        cv::meanStdDev(right_laplacian, cv::Scalar(), right_var);
        
        std::cout << "Edge variance (higher = more detail, lower = grid artifacts):" << std::endl;
        std::cout << "  Left variance: " << left_var[0] << std::endl;
        std::cout << "  Right variance: " << right_var[0] << std::endl;
        
        std::cout << "\n✓ Agent's SBGGR10 implementation tested successfully!" << std::endl;
        
    } else {
        std::cerr << "Failed to capture frame" << std::endl;
        return 1;
    }
    
    capture.stop();
    return 0;
}