# Unlook 3D Scanner - Stereo Calibration Guide

## Table of Contents
1. Introduction
2. Hardware Requirements
3. Step-by-Step Calibration Workflow
4. Best Practices
5. Common Issues

---

## Introduction

### What is Stereo Calibration?

Stereo calibration is the process of determining the precise geometric relationship between two cameras in a stereo vision system. It involves:

- **Camera Intrinsics**: Focal length, principal point, and lens distortion parameters for each camera
- **Camera Extrinsics**: Rotation and translation (baseline) between the two cameras
- **Rectification**: Epipolar geometry alignment to ensure corresponding points lie on the same horizontal line

### Why is Calibration Critical?

Unlook's 0.005mm precision target depends entirely on accurate calibration. Poor calibration results in:
- Depth measurement errors
- Misaligned stereo pairs
- Invalid 3D point clouds
- Reduced mesh quality

### When Should You Recalibrate?

Recalibrate immediately in these situations:
- After mechanical adjustments to camera mounting
- After lens replacement or repair
- After hardware synchronization changes
- When depth accuracy degrades noticeably

Recommended periodic recalibration:
- Every 3-6 months for continuous operation
- After significant temperature changes (>20°C)
- If error metrics exceed thresholds

### Quality Targets

Aim for these metrics during calibration:
- **RMS Reprojection Error**: < 0.3 pixels (PASS), < 0.6 pixels (WARNING)
- **Baseline Error**: < 0.5 mm (PASS), < 1.0 mm (WARNING)
- **Epipolar Error**: < 0.5 pixels (PASS)
- **Valid Image Pairs**: >= 30 (minimum 45+ recommended)

---

## Hardware Requirements

### Cameras and Synchronization
- Two synchronized IMX296 global shutter cameras
- Hardware synchronization via XVS/XHS connections
- Rigid mechanical mounting (critical for stability)
- Clean camera lenses (check for dust and condensation)

### Calibration Pattern

**Recommended: ChArUco 7x10 (with ArUco markers)**

Pattern specifications:
- **Type**: ChArUco with ArUco markers
- **Rows**: 7
- **Columns**: 10
- **Square Size**: 24.0 mm (CRITICAL - must be accurate)
- **ArUco Marker Size**: 17.0 mm
- **Dictionary**: DICT_4X4_250
- **Material**: Rigid backing (foam board or plexiglass recommended)

**Printing Instructions**:

1. Print at 100% scale (no scaling)
2. Verify square dimensions with digital calipers
   - Each square should be exactly 24mm × 24mm
   - Tolerance: ±0.5mm maximum
3. Mount on rigid backing material (not paper)
4. Ensure pattern is perfectly flat (check with straightedge)
5. Protect pattern from warping (store in flat, dry place)

**Alternative Patterns**:
- **Checkerboard (Classic)**: 9×6 pattern, 25mm squares
  - Faster detection, less robust to occlusion
- **Circle Grid**: Asymmetric pattern
  - Subpixel accuracy excellent, slower detection

### Lighting Conditions
- Adequate ambient lighting (natural or LED)
- VCSEL LED automatically enables during capture (do not disable)
- Avoid harsh shadows on pattern
- Diffuse lighting preferred over point sources

### Work Surface
- Flat, stable surface for pattern movement
- Minimum 1.5m × 1m working area
- Camera mount should not move during capture
- Temperature stable (avoid direct sunlight)

---

## Step-by-Step Calibration Workflow

### Step 1: Prepare Calibration Pattern

**You will need:**
- Printed ChArUco 7×10 pattern on rigid backing
- Digital calipers to verify dimensions
- Holder or mounting method (hands, tripod, etc.)

**Verification procedure:**

1. Measure pattern dimensions with calipers
   - Check multiple squares on pattern
   - All measurements should be 24.0 ± 0.5 mm
   - Record actual measurements for reference

2. Check pattern integrity
   - Ensure no warping or bending
   - Verify all corners are sharp (no pixelation)
   - Check that ArUco markers are distinct and visible

3. Clean pattern surface
   - Remove dust with soft cloth
   - Ensure good contrast between black and white

**What happens if dimensions are wrong?**
- Square size error causes proportional calibration error
- 1mm error in square size = 1mm error in baseline calculation
- CRITICAL: Always verify with calipers

### Step 2: Open Calibration Interface

1. Launch Unlook Scanner:
   ```bash
   unlook
   ```

2. Main window displays with multiple tabs

3. Click **Calibration** button
   - Opens tabbed calibration interface
   - Two tabs: "Dataset Capture" and "Dataset Processing"

4. "Dataset Capture" tab is active by default

### Step 3: Configure Pattern Settings

In the "Dataset Capture" tab, configure these parameters:

#### Pattern Type
- Select **"ChArUco (Recommended)"** from dropdown
- Alternatives: Checkerboard, Circle Grid

#### Pattern Dimensions
- **Rows**: 7 (match your printed pattern)
- **Columns**: 10 (match your printed pattern)
- **Square Size**: 24.0 mm (CRITICAL - verify with calipers!)
- **ArUco Marker Size**: 17.0 mm

#### Verification Checklist
```
[  ] Pattern type set to ChArUco
[  ] Rows set to 7
[  ] Columns set to 10
[  ] Square size set to 24.0 mm (VERIFIED WITH CALIPERS!)
[  ] ArUco size set to 17.0 mm
[  ] Pattern is flat and clean
[  ] Lighting is adequate (no harsh shadows)
```

### Step 4: Position Pattern and Check Detection

**Before starting capture:**

1. Hold pattern in front of cameras
2. Watch both preview windows
3. Look for pattern detection indicators:
   - **Green ✓ "Pattern Detected"**: Good, ready to capture
   - **Red ✗ "Pattern Not Detected"**: Reposition pattern
   - Corner count shows: "✓ Detected: 70 corners"

4. When both cameras show green checkmarks, you're ready

**If pattern is not detected:**
- Move closer or farther (typical range: 30-80cm)
- Improve lighting
- Ensure pattern is fully visible in both frames
- Check pattern configuration matches printed pattern

### Step 5: Start Automated Capture

1. Click **"Start Dataset Capture (50 pairs)"** button
   - VCSEL LED automatically enables
   - System captures FIRST frame immediately
   - Progress bar shows: "Captured: 0 / 50 pairs"

2. Status label shows:
   - "Capturing... move checkerboard between captures"
   - Timer: 5 seconds until next capture

### Step 6: Capture 50 Image Pairs

**Automated capture workflow:**

```
Action                          Time
┌─────────────────────────────────────┐
│ User clicks "Start Capture"         │ 0s
├─────────────────────────────────────┤
│ Frame 0 captured immediately        │ 0s
│ Status: "Captured 0/50 - move..."   │ 0s
├─────────────────────────────────────┤
│ Wait 5 seconds                      │ 0-5s
│ User moves pattern to new position  │ 1-4s
├─────────────────────────────────────┤
│ Frame 1 auto-captured               │ 5s
│ Status: "Captured 1/50 - move..."   │ 5s
├─────────────────────────────────────┤
│ Repeat 5s wait + move cycle         │ 5-250s
├─────────────────────────────────────┤
│ Frame 49 captured                   │ 245s
│ Capture complete!                   │ 250s
└─────────────────────────────────────┘

Total time: ~4-5 minutes for 50 pairs
```

**Best practices for quality calibration:**

1. **Maximize Position Variety**
   - Center, top-left, top-right, bottom-left, bottom-right
   - Close to cameras (30-50cm), far from cameras (60-80cm)
   - Different distances help calibrate depth accuracy

2. **Maximize Angle Variety**
   - Flat perpendicular to cameras (0°)
   - Tilted left/right: ±15° to ±30°
   - Tilted up/down: ±15° to ±30°
   - Rotated clockwise/counterclockwise: some rotation

3. **Field of View Coverage**
   - Place pattern at all corners of frame
   - Move to edges and center
   - Ensure all areas of image are represented

4. **Motion and Stability**
   - Keep pattern steady during each 5-second capture window
   - Move smoothly between positions
   - Avoid rapid jitter

5. **Quality Monitoring**
   - Watch detection status in preview
   - Red indicators mean pattern not detected in that image
   - Low corner counts (< 60/70) indicate poor quality
   - Reposition if frequently seeing red ✗

**Progress Indication:**

Progress bar updates every 5 seconds:
```
Captured: 0 / 50 pairs    [████░░░░░░░░░░░░░░░░]  0%
Captured: 10 / 50 pairs   [████████░░░░░░░░░░░░] 20%
Captured: 25 / 50 pairs   [████████████████░░░░] 50%
Captured: 40 / 50 pairs   [█████████████████████] 80%
Captured: 50 / 50 pairs   [████████████████████] 100% COMPLETE
```

### Step 7: Dataset Capture Complete

When 50 pairs are successfully captured:

1. VCSEL LED automatically turns off
2. Progress bar shows 100%
3. Status label shows: "Dataset capture complete!"
4. Popup appears with dataset location:
   ```
   Dataset saved to:
   /unlook_calib_dataset/dataset_20250104_153000

   Proceed to Processing tab to calibrate.
   ```

5. Dataset contains:
   - `/unlook_calib_dataset/dataset_YYYYMMDD_HHMMSS/left/frame_XXX.png` (50 images)
   - `/unlook_calib_dataset/dataset_YYYYMMDD_HHMMSS/right/frame_XXX.png` (50 images)
   - `/unlook_calib_dataset/dataset_YYYYMMDD_HHMMSS/dataset_info.json` (metadata)

### Step 8: Switch to Dataset Processing Tab

1. Click **"Dataset Processing"** tab
2. Latest dataset automatically loaded
3. Display shows:
   - Path: `/unlook_calib_dataset/dataset_20250104_153000`
   - Timestamp: `20250104_153000`
   - Image pairs: `50`
   - Pattern: `ChArUco 7×10, 24mm`

### Step 9: Process Dataset

Click **"Process Dataset"** button to start calibration:

**Automatic processing steps:**

```
Step 1: Loading dataset JSON and images
        ████░░░░░░░░░░░░░░░░░  10%

Step 2: Detecting calibration patterns
        ████████░░░░░░░░░░░░░  20%

Step 3: Filtering valid image pairs
        ████████████░░░░░░░░░  30%

Step 4: Calibrating left camera intrinsics
        ████████████████░░░░░  40%

Step 5: Calibrating right camera intrinsics
        ████████████████████░  50%

Step 6: Performing stereo calibration
        ████████████████████░░ 60%

Step 7: Computing rectification transforms
        ████████████████████░░ 70%

Step 8: Generating rectification maps
        ████████████████████░░ 80%

Step 9: Validating calibration quality
        ████████████████████░░ 90%

Step 10: Computing baseline and epipolar errors
        ████████████████████░░ 100%

Status: Calibration completed in 42.3 seconds
```

**Processing time**: Typically 30-60 seconds depending on hardware

**Log output shows**:
- Each processing step with timestamps
- Pattern detection results for each image pair
- Calibration parameters being computed
- Quality metrics as they are calculated

### Step 10: Review Calibration Results

After processing completes, results display shows:

#### Quality Metrics Display

```
RMS Reprojection Error: 0.28 pixels
Status: ✓ PASS (threshold: 0.3 px)

Baseline Error: 0.12 mm
Status: ✓ PASS (threshold: 0.5 mm)

Epipolar Error: 0.19 pixels
Status: ✓ PASS (threshold: 0.5 px)

Valid Image Pairs: 48 / 50
Status: ✓ PASS (minimum: 30)
```

#### Interpretation Guide

**RMS Reprojection Error**
- Measures how well detected corners match calibration model
- **<0.3 px**: Excellent calibration
- **0.3-0.6 px**: Acceptable (warning)
- **>0.6 px**: Poor quality, recalibrate

**Baseline Error**
- Difference between measured and expected baseline (70mm)
- **<0.5 mm**: Excellent mechanical alignment
- **0.5-1.0 mm**: Acceptable (check mechanical mount)
- **>1.0 mm**: Critical issue, check camera mounting

**Epipolar Error**
- Measures alignment of stereo pairs
- **<0.5 px**: Excellent rectification
- **0.5-1.0 px**: Acceptable
- **>1.0 px**: Poor rectification

**Valid Image Pairs**
- How many image pairs had successful pattern detection
- Need minimum 30 valid pairs
- Ideally 45+ for robust calibration

#### Quality Summary Table

Statistics table shows detailed information:

```
Parameter                              Value
─────────────────────────────────────────────
Image Width                            1280 px
Image Height                           720 px
Total Captured Pairs                   50
Valid Detection Pairs                  48
Pattern Type                           ChArUco 7×10
Square Size                            24.0 mm

Mean Reprojection Error (Left)         0.26 px
Mean Reprojection Error (Right)        0.29 px
Max Reprojection Error                 0.87 px
RMS Reprojection Error                 0.28 px

Mean Epipolar Error                    0.15 px
Max Epipolar Error                     0.42 px

Baseline (measured)                    70.12 mm
Baseline (expected)                    70.00 mm
Baseline Error                         0.12 mm
Baseline Error (%)                     0.17%

Calibration Duration                   42.3 seconds
```

### Step 11: Calibration Auto-Applied

If calibration quality PASSES:

1. **Automatic Actions**:
   - Calibration saved to: `/unlook_calib/calib-20250104_153000.yaml`
   - Set as system default (symlink update)
   - Previous calibration backed up automatically

2. **YAML File Contents**:
   ```yaml
   calibration_date: "2025-01-04T15:30:00"
   dataset_timestamp: "20250104_153000"

   # Camera intrinsics
   camera_matrix_left:
     fx: 1220.5
     fy: 1221.2
     cx: 640.1
     cy: 360.3

   distortion_coeffs_left:
     k1: -0.15
     k2: 0.08
     p1: 0.001
     p2: -0.002
     k3: -0.02

   # ... (right camera intrinsics)

   # Stereo geometry
   rotation_matrix: [...]
   translation_vector:
     tx: -70.12    # Baseline in mm
     ty: 0.15
     tz: -0.08

   # Quality metrics
   rms_reprojection_error: 0.28
   baseline_error_mm: 0.12
   quality_passed: true
   ```

3. **System Integration**:
   - Scanner immediately uses new calibration
   - `unlook_calib/default.yaml` symlink updated
   - Depth measurements now use new parameters

### Step 12: Verify Calibration

To verify calibration is working correctly:

1. **Return to main interface**: Close or minimize calibration widget

2. **Test depth measurement**:
   - Use "Depth Test" tab
   - Scan a known object (cube, sphere, cylinder)
   - Measure dimensions and compare to actual values

3. **Check 3D point cloud**:
   - Points should form clean, coherent 3D structures
   - No scattered outliers
   - Good spatial coherence

4. **Mesh quality**:
   - Surfaces should be smooth without distortion
   - No gaps or holes unless due to occlusion

---

## Best Practices

### Before Calibration Session

- [ ] Verify pattern dimensions with calipers (24.0 ± 0.5 mm)
- [ ] Ensure cameras are rigidly mounted (check all screws)
- [ ] Clean camera lenses with soft, dry cloth
- [ ] Check for lens damage or condensation
- [ ] Allow scanner to reach operating temperature
- [ ] Disable other applications to free system resources
- [ ] Ensure good lighting conditions

### During Capture

- [ ] Wait for green ✓ detection indicator before starting
- [ ] Capture first frame, then move pattern for each frame
- [ ] Move smoothly between positions (no jerky motion)
- [ ] Cover entire field of view (corners, edges, center)
- [ ] Include varied angles (flat, tilted, rotated)
- [ ] Vary distances from cameras
- [ ] Monitor detection status - red ✗ = poor quality
- [ ] Complete all 50 captures without interruption

### Pattern Coverage Strategy

Recommended capture positions (50 total):

```
Frame   Position              Angle          Distance
1-5     Center               0°              50cm
6-10    Top-left            +15°/-30°        40cm
11-15   Top-right           +15°/+30°        40cm
16-20   Bottom-left         -15°/-30°        60cm
21-25   Bottom-right        -15°/+30°        60cm
26-30   Left edge           ±15° tilt        50cm
31-35   Right edge          ±15° tilt        50cm
36-40   Close to camera     0° to ±30°       35cm
41-45   Far from camera     0° to ±30°       75cm
46-50   Mixed positions     Various          Various
```

### After Calibration

- [ ] Review quality metrics
- [ ] Check that RMS error < 0.3 pixels
- [ ] Check that baseline error < 0.5 mm
- [ ] If PASS: calibration is complete
- [ ] If WARNING or FAIL: recapture with improved technique
- [ ] Test on known objects to verify accuracy
- [ ] Store calibration reference data

### Maintenance Schedule

**Immediately recalibrate after:**
- Mechanical adjustment to camera mount
- Lens replacement
- Hardware synchronization changes
- Collision or physical impact

**Periodic recalibration (recommended):**
- Every 3 months for continuous operation
- After temperature extremes (>20°C change)
- If depth accuracy noticeably degrades
- Seasonal environmental changes

---

## Troubleshooting Quick Reference

| Issue | Symptom | Solution |
|-------|---------|----------|
| Pattern not detected | Red ✗ indicator | Reposition pattern, improve lighting |
| Low valid pairs | <30 valid from 50 captured | Recapture with better positioning variety |
| High RMS error (>0.6px) | ⚠ WARNING status | Check pattern dimensions with calipers |
| High baseline error (>1mm) | CRITICAL status | Check mechanical mounting, measure baseline |
| Processing hangs | No progress after 5 min | Kill process, check disk space, increase RAM |
| Depth measurements wrong | Known object measures incorrectly | Verify calibration applied, check YAML file |

For detailed troubleshooting, see [CALIBRATION_TROUBLESHOOTING.md](./CALIBRATION_TROUBLESHOOTING.md)

---

## System Calibration Directories

**Dataset storage:**
```
/unlook_calib_dataset/
├── dataset_20250104_153000/
│   ├── left/
│   │   ├── frame_000.png
│   │   ├── frame_001.png
│   │   └── ... (50 total)
│   ├── right/
│   │   ├── frame_000.png
│   │   ├── frame_001.png
│   │   └── ... (50 total)
│   └── dataset_info.json
└── dataset_20250105_091500/
    └── ...
```

**Calibration files:**
```
/unlook_calib/
├── calib-20250104_153000.yaml (timestamp format)
├── calib-20250105_091500.yaml
├── default.yaml (symlink to latest)
└── calib-TIMESTAMP-map-left-x.bin (rectification map)
```

**Access permissions:**
```bash
# Calibration directories have write permissions
ls -la /unlook_calib_dataset
ls -la /unlook_calib

# System default symlink
ls -la /unlook_calib/default.yaml
```

---

## Frequently Asked Questions

**Q: How long does calibration take?**
A: Capture is ~5 minutes, processing is 30-60 seconds. Total: ~6-7 minutes.

**Q: Can I use a different pattern?**
A: Yes, but ChArUco is recommended for robustness. Configure pattern dimensions to match your printed pattern.

**Q: What if pattern dimensions are wrong?**
A: Incorrect dimensions cause proportional calibration errors. ALWAYS verify with calipers.

**Q: Can I move the cameras after calibration?**
A: No, any mechanical change invalidates calibration. Recalibrate immediately.

**Q: How often should I recalibrate?**
A: Every 3-6 months for continuous operation, or after mechanical changes.

**Q: What's the "Baseline"?**
A: Distance between camera centers (70mm). Error > 1mm indicates mechanical problems.

**Q: Why is RMS error important?**
A: It measures calibration accuracy. Target < 0.3 pixels for professional results.

**Q: Can I recalibrate without starting from scratch?**
A: Start new dataset capture. Previous calibrations are backed up automatically.

---

## Next Steps

1. Print and verify ChArUco 7×10 pattern
2. Mount Unlook scanner securely
3. Open Calibration interface in Unlook application
4. Perform dataset capture (5 minutes)
5. Process dataset (1 minute)
6. Review quality metrics
7. Test on known objects
8. Ready for production scanning

For technical details on parameters, see [CALIBRATION_PARAMETERS.md](./CALIBRATION_PARAMETERS.md)

For JSON schema documentation, see [CALIBRATION_JSON_SCHEMA.md](./CALIBRATION_JSON_SCHEMA.md)

---

**Document Version**: 1.0
**Last Updated**: 2025-01-04
**Status**: Production Ready
