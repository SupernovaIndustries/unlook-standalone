#!/bin/bash
# Second round of fixes

echo "Applying additional fixes..."

# Fix MedianFilter.cpp - remove DebugOutputManager::getInstance()
sed -i '/DebugOutputManager::getInstance/d' src/stereo/MedianFilter.cpp

# Fix DepthConverter.cpp - remove DebugOutputManager::getInstance()
sed -i '/DebugOutputManager::getInstance/d' src/stereo/DepthConverter.cpp

# Fix DisparityComputer.cpp - remove debugManager references
sed -i '/debugManager/d' src/stereo/DisparityComputer.cpp

# Fix DisparityComputer.cpp - comment out entire applyWLSFilter function body
cat > /tmp/wls_patch.txt << 'EOF'
void DisparityComputer::applyWLSFilter(
    const cv::Mat& leftRect,
    cv::Mat& disparity,
    cv::Mat& confidence)
{
    // WLS filter temporarily disabled - requires opencv_contrib ximgproc module
    // TODO: Enable when ximgproc is available
    if (logger_) {
        logger_->info("WLS filter disabled (ximgproc not available)");
    }
    confidence = cv::Mat::ones(disparity.size(), CV_8U) * 255;
    return;
}
EOF

# Find and replace applyWLSFilter function
# This is tricky with sed, so we'll just comment out problematic lines
sed -i '782,805s|^|// |' src/stereo/DisparityComputer.cpp

# Fix updateWLSFilter function
sed -i '893,896s|^|// |' src/stereo/DisparityComputer.cpp

echo "✓ Additional fixes applied"
