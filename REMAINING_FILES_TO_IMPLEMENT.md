# REMAINING FILES TO IMPLEMENT - CRITICAL

These files have STUBS that allow compilation but NEED FULL IMPLEMENTATION:

## 1. src/calibration/StereoCalibrationProcessor.cpp
**Status:** Needs full OpenCV stereo calibration implementation
**Lines:** ~800 lines
**Critical Functions:**
- `calibrateFromDataset()` - Main calibration pipeline
- `detectPattern()` - Pattern detection dispatch
- `calibrateStereo()` - OpenCV stereo calibration
- `validateCalibration()` - Quality checks with RMS/baseline validation
- `saveCalibrationYAML()` - Export complete calibration to YAML
- `loadCalibrationYAML()` - Load existing calibration

**Implementation Notes:**
- Use `cv::stereoCalibrate()` for main calibration
- Use `cv::stereoRectify()` for rectification transforms
- Use `cv::initUndistortRectifyMap()` for rectification maps
- Validate RMS error < 0.3px, baseline error < 0.5mm
- Save to `/unlook_calib/calib-TIMESTAMP.yaml`

## 2. src/calibration/PatternDetector.cpp  
**Status:** Already exists but needs verification
**Lines:** ~300 lines
**Critical Functions:**
- `detect()` - Main detection function
- `detectCharuco()` - ChArUco pattern with ArUco markers
- `detectCheckerboard()` - Classic checkerboard
- `drawCharucoOverlay()` - Visual feedback

**Implementation Notes:**
- Use `cv::aruco::detectMarkers()` for ArUco detection
- Use `cv::aruco::interpolateCornersCharuco()` for ChArUco corners
- Use `cv::findChessboardCorners()` for checkerboard
- Draw corners with `cv::drawChessboardCorners()` or `cv::aruco::drawDetectedCornersCharuco()`

## BUILD STATUS

**Files Created:**
- ✓ All header files (5 headers)
- ✓ calibration_widget.cpp (35 lines)
- ✓ dataset_capture_widget.cpp (350 lines - functional)
- ✓ dataset_processing_widget.cpp (220 lines - functional stub)

**Files Needed:**
- ❌ StereoCalibrationProcessor.cpp (800 lines) - CRITICAL
- ❌ PatternDetector.cpp (verify existing or implement 300 lines)

**Integration:**
- ✓ main_window.ui modified (calibration button added)
- ✓ main_window.hpp modified (calibration widget member added)
- ✓ main_window.cpp modified (calibration navigation added)
- ✓ src/gui/CMakeLists.txt modified (new files added)
- ❌ src/calibration/CMakeLists.txt needs update
- ❌ Top-level CMakeLists.txt needs nlohmann-json dependency

## IMPLEMENTATION PRIORITY

**HIGH PRIORITY (Required for compilation):**
1. Create minimal stub for StereoCalibrationProcessor.cpp
2. Update src/calibration/CMakeLists.txt
3. Check/fix PatternDetector.cpp

**MEDIUM PRIORITY (Required for functionality):**
4. Implement full StereoCalibrationProcessor::calibrateFromDataset()
5. Implement StereoCalibrationProcessor::saveCalibrationYAML()
6. Implement pattern detection logic

**LOW PRIORITY (Polish):**
7. Add detailed logging
8. Add progress callbacks
9. Add quality validation UI improvements

## NEXT STEPS FOR USER/DEVELOPER

1. Run build to see if stubs compile:
   ```bash
   cd /home/alessandro/unlook-standalone
   ./build.sh --clean -j4
   ```

2. If compilation succeeds, implement full calibration logic in:
   - `src/calibration/StereoCalibrationProcessor.cpp`

3. Test calibration workflow:
   - Run `unlook`
   - Click "CALIBRATION" button
   - Capture dataset (50 pairs)
   - Process dataset
   - Verify calibration quality

## SUMMARY

**Status:** 80% complete
- GUI framework: ✓ COMPLETE
- Widget integration: ✓ COMPLETE
- Calibration backend: ⚠ STUBS ONLY - needs full implementation
- Build system: ⚠ Partially updated

**Estimated time to complete:** 2-4 hours for full calibration algorithm implementation
