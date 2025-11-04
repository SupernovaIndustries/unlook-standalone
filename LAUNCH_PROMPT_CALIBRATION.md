# 🚀 LAUNCH COMMAND FOR CALIBRATION SYSTEM

Leggi il file `/home/alessandro/unlook-standalone/MEGA_PROMPT_CALIBRATION_SYSTEM.md` che contiene le specifiche complete per il sistema di calibrazione stereo professionale.

## CRITICAL REQUIREMENTS:

### GUI Changes:
- **REMOVE:** "Face Enrollment" button from main window
- **ADD:** "Calibration" button → opens CalibrationWidget
- **NEW WIDGETS:**
  - CalibrationWidget (tab container)
  - DatasetCaptureWidget (dual preview + auto-capture)
  - DatasetProcessingWidget (processing + validation)

### Features to Implement:
1. **Dataset Capture:**
   - Dual camera preview with VCSEL LED active
   - Real-time pattern detection overlay (OpenCV markers)
   - Pattern config: Checkerboard, ChArUco (recommended), Circle Grid
   - Input controls: Qt widgets (NO keyboard input)
   - Auto-capture: 50 pairs with 5 second delay
   - Save to: `/unlook_calib_dataset/dataset_TIMESTAMP/`
   - JSON metadata with all info

2. **Dataset Processing:**
   - Auto-load latest dataset
   - Full stereo calibration (OpenCV + validation)
   - Real-time log output
   - Quality checks: RMS error, baseline, epipolar
   - Save to: `/unlook_calib/calib-TIMESTAMP.yaml`
   - Auto-set as system default
   - Display statistics in GUI

3. **Backend:**
   - StereoCalibrationProcessor with full pipeline
   - PatternDetector with ChArUco support
   - CalibrationValidator with quality criteria
   - Complete YAML output with ALL parameters

### Launch Agents IN PARALLEL:
1. **stereo-calibration-specialist** - Backend calibration
2. **ux-ui-design-architect** - GUI widgets (remove Face Enrollment, add Calibration)
3. **camera-sync-manager** - LED integration
4. **technical-documentation-specialist** - Docs

After completion:
- **code-integrity-architect** - Review
- **cmake-build-system-architect** - Build system (install nlohmann-json if needed)
- **BUILD** - `./build.sh --clean -j4`
- **testing-validation-framework** - Tests

### Dependencies to Install if Needed:
```bash
sudo apt-get install -y nlohmann-json3-dev
```

### Target:
- RMS error: < 0.3 pixels
- Baseline error: < 0.5mm
- Processing time: < 60 seconds
- User workflow: < 5 minutes total

**NO PLACEHOLDER CODE. NO FAKE CODE. EVERYTHING REAL AND WORKING.**
