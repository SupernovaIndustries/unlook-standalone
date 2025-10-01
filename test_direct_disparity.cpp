// Test program for direct disparity-to-3D conversion
#include <iostream>
#include <opencv2/opencv.hpp>
#include <fstream>

int main() {
    std::cout << "\n========================================" << std::endl;
    std::cout << "Direct Disparity to 3D Test" << std::endl;
    std::cout << "========================================\n" << std::endl;

    // Check if debug disparity files exist from previous runs
    std::string debugDir = "/home/alessandro/unlook_debug/";
    std::cout << "Checking for debug disparity files in: " << debugDir << std::endl;

    // List available disparity files
    std::string cmd = "ls -la " + debugDir + "*/disparity_raw.exr 2>/dev/null | head -5";
    system(cmd.c_str());

    std::cout << "\nTo test the direct conversion:" << std::endl;
    std::cout << "1. Run the GUI: ./build/src/gui/unlook_scanner" << std::endl;
    std::cout << "2. Enable 'computePointCloud' in settings" << std::endl;
    std::cout << "3. Capture a depth scan" << std::endl;
    std::cout << "4. Check /tmp/direct_disparity_conversion.log for detailed output" << std::endl;
    std::cout << "5. Check /tmp/direct_pointcloud_*.ply for the generated point cloud" << std::endl;

    std::cout << "\nExpected results:" << std::endl;
    std::cout << "- ~1 million 3D points (vs previous 11 points)" << std::endl;
    std::cout << "- Direct conversion from disparity bypasses lossy depth map" << std::endl;
    std::cout << "- Professional stereo formula: depth = (baseline * focal) / disparity" << std::endl;

    return 0;
}