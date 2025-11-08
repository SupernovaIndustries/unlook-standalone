#!/bin/bash
# Quick fix for compilation errors

echo "Fixing compilation errors..."

# Fix 1: COLORMAP_TURBO in DebugOutputManager.cpp
sed -i 's/cv::COLORMAP_TURBO/20/g' src/stereo/DebugOutputManager.cpp

# Fix 2: String concatenation in DisparityComputer.cpp (lines 49-52)
sed -i '49,52s|logger_->info("  Method: " +|logger_->info(std::string("  Method: ") +|' src/stereo/DisparityComputer.cpp

# Fix 3: String concatenation (lines 189-192)
sed -i '189,192s|logger_->info("  Method: " +|logger_->info(std::string("  Method: ") +|' src/stereo/DisparityComputer.cpp

# Fix 4: Comment out WLS filter (not available without ximgproc)
# Disable WLS filter in DisparityComputer.hpp
sed -i 's|bool useWLSFilter = true;|bool useWLSFilter = false; // DISABLED: requires opencv_contrib ximgproc|' include/unlook/stereo/DisparityComputer.hpp

# Fix 5: Comment out ximgproc include
sed -i 's|#include <opencv2/ximgproc.hpp>|// #include <opencv2/ximgproc.hpp> // DISABLED: not available|' src/stereo/DisparityComputer.cpp

# Fix 6: Remove DebugOutputManager::getInstance() calls (not a singleton)
sed -i 's|DebugOutputManager::getInstance()|debugManager|g' src/stereo/DisparityComputer.cpp

# Fix 7: Disable WLS filter member variable (line 161)
sed -i '161s|cv::Ptr<cv::ximgproc::DisparityWLSFilter> wlsFilter_;|// WLS filter disabled - requires opencv_contrib ximgproc\n    // cv::Ptr<cv::ximgproc::DisparityWLSFilter> wlsFilter_;|' include/unlook/stereo/DisparityComputer.hpp

echo "✓ Compilation fixes applied"
