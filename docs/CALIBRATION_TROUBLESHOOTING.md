# Stereo Calibration Troubleshooting Guide

## Quick Diagnosis Table

| Symptom | First Check | Solution | See Section |
|---------|-------------|----------|-------------|
| Pattern not detected | Camera focus, lighting | Reposition, improve light | Pattern Detection Issues |
| Only 15-20 valid pairs | Capture technique | Recapture with variety | Low Valid Pairs |
| RMS error 0.8+ pixels | Pattern dimensions | Verify with calipers | High RMS Error |
| Baseline error >1mm | Mechanical mount | Check camera mount | High Baseline Error |
| Processing hangs/crashes | System resources | Free RAM, restart | Processing Failures |
| Depth measurements wrong | Calibration applied | Check system default file | Verification Issues |

---

## Common Issues and Solutions

### Pattern Detection Issues

#### Symptom: Red ✗ "Pattern Not Detected" indicator

**What this means:**
- Pattern detection algorithm failed to recognize the calibration pattern
- Likely causes: focus, lighting, pattern positioning, pattern configuration

**Diagnosis Steps:**

1. **Check Camera Focus**
   ```
   Is the pattern in sharp focus?
   - Pattern too close (<30cm): May be out of focus
   - Pattern too far (>80cm): May be blurry
   - Lens may have fixed focus setting
   ```

2. **Check Pattern Visibility**
   ```
   Is the entire pattern visible?
   - Check both camera preview windows
   - Pattern should not be cut off at edges
   - Pattern should fill at least 40% of frame
   ```

3. **Check Lighting Conditions**
   ```
   Is lighting adequate?
   - Watch for harsh shadows on pattern
   - Ensure even illumination across pattern
   - Avoid backlighting (light behind pattern)
   - Turn on room lights if indoors
   ```

4. **Check Pattern Configuration**
   ```
   Does configuration match printed pattern?
   - Rows: 7 (count rows in pattern)
   - Cols: 10 (count columns)
   - Pattern type: ChArUco
   - Compare to printed pattern
   ```

**Solutions:**

**Solution 1: Adjust Focus**
- Move pattern closer or farther (typical range: 30-80cm)
- Find distance where pattern is sharp
- Once focused, maintain similar distance for all captures

**Solution 2: Improve Lighting**
- Increase room illumination
- Avoid direct shadows on pattern
- Use diffuse light sources
- VCSEL LED automatically enables (do not disable)
- Try different angles to pattern

**Solution 3: Reposition Pattern**
- Ensure entire pattern is visible in both cameras
- Move pattern more toward center of frame
- Reduce tilt angle if >45 degrees
- Check that pattern is flat (not warped)

**Solution 4: Verify Pattern Settings**
```cpp
// Check these match your printed pattern:
Pattern Type: ChArUco (SELECTED?)
Rows: 7          (CORRECT?)
Columns: 10      (CORRECT?)
Square Size: 24mm (VERIFIED WITH CALIPERS?)
```

**Solution 5: Check Pattern Quality**
- Verify pattern is flat and not bent
- Check that black/white contrast is high
- Ensure ArUco markers are distinct
- Pattern should not be faded or worn

**Prevention:**
- Test pattern detection before starting capture session
- Move pattern around to find optimal focus distance
- Note this distance for calibration session
- Maintain consistent distance during capture

---

### Low Valid Image Pairs

#### Symptom: Processing shows "Valid pairs: 18/50 (need >= 30)"

**What this means:**
- Pattern was not successfully detected in many captured images
- Need at least 30 valid pairs for calibration to proceed
- This is critical - calibration will FAIL with insufficient valid pairs

**Root Causes Analysis:**

```
Total Captures: 50
Valid Detections: 18
Lost Pairs: 32 (64% failure rate)

Likely causes in order of probability:
1. Focus issues (30% of cases)
   - Pattern not in sharp focus in many captures
   - Focal distance drifted during capture

2. Pattern positioning (25% of cases)
   - Pattern moved outside frame in some captures
   - Pattern partially cut off at image edges
   - Pattern became tilted beyond recognition angle

3. Lighting variations (20% of cases)
   - Light changed between captures
   - Shadows appeared on pattern
   - Inconsistent illumination

4. Pattern configuration mismatch (15% of cases)
   - Entered wrong rows/cols/square size
   - Pattern type doesn't match printed pattern

5. Camera sync issues (10% of cases)
   - One camera lagging behind
   - Sync hardware malfunction
   - Check XVS/XHS connections
```

**Diagnosis Steps:**

1. **Review Dataset Images**
   ```bash
   # Open dataset folder to inspect images
   /unlook_calib_dataset/dataset_YYYYMMDD_HHMMSS/left/
   /unlook_calib_dataset/dataset_YYYYMMDD_HHMMSS/right/

   # Look at frames manually:
   # - Frame 0-9: Check focus and positioning
   # - Frame 15-25: Middle captures
   # - Frame 45-49: End captures

   # Common issues in images:
   - Blurry images = focus problem
   - Pattern partially cut off = positioning
   - Different lighting = inconsistent conditions
   - Faded/dark images = lighting problem
   ```

2. **Check Pattern Settings**
   - Verify rows, columns, square size
   - Compare to your printed pattern
   - Check that ArUco size is correct (17mm)

3. **Analyze Failure Pattern**
   ```
   Where do failures occur?
   - If fails in later frames (30+): Focus drift
   - If random failures throughout: Lighting issue
   - If specific frames fail: Check those images
   - If all fails: Pattern configuration mismatch
   ```

**Solutions:**

**Solution 1: Recapture with Better Technique**

Next attempt focus on:
1. **Consistent Focus Distance**
   - Measure distance from camera to pattern
   - Keep distance within 5cm range throughout
   - Use fixed focal distance (not varying)

2. **Consistent Positioning**
   - Keep pattern fully visible in both cameras
   - Avoid having pattern partially cut off
   - Don't tilt pattern beyond ±45 degrees

3. **Consistent Lighting**
   - Use same room lighting for entire session
   - Avoid moving between light/dark areas
   - Keep pattern in even illumination

4. **Quality Monitoring**
   - Watch preview while capturing
   - After each capture, check image quality
   - If image is blurry, note and maintain focus

**Solution 2: Validate Pattern Configuration**
```
Before recapturing:
1. Count rows in printed pattern = ?
2. Count columns in printed pattern = ?
3. Measure square size with calipers = ? mm
4. Check pattern type = ChArUco?

Update configuration if needed:
Pattern Type: [ChArUco selected?]
Rows: [match printed pattern]
Columns: [match printed pattern]
Square Size: [measured value]
```

**Solution 3: Start Fresh**
- Delete previous failed dataset
- Clear any memory issues
- Start new capture session
- Go slowly and carefully

**Prevention for Next Attempt:**
- Test pattern detection BEFORE starting capture
- Move pattern around slowly to find optimal setup
- Practice the 5-capture cycle manually first
- Ensure stable lighting before capturing
- Use dedicated mounting (not hands) if possible

---

### High RMS Reprojection Error

#### Symptom: Processing shows RMS error > 0.6 pixels (WARNING/FAIL)

**What this means:**
- Detected pattern corners don't match the geometric model
- Could indicate poor pattern quality, incorrect dimensions, or capture issues
- Most common cause: Incorrect square size

**RMS Error Severity Scale:**
```
0.0-0.2 px:  EXCELLENT (professional quality)
0.2-0.3 px:  VERY GOOD (recommended target)
0.3-0.5 px:  GOOD (acceptable)
0.5-0.8 px:  ACCEPTABLE (warning zone)
0.8-1.2 px:  POOR (should recalibrate)
>1.2 px:     FAIL (invalid calibration)
```

**Root Causes (in order of probability):**

1. **Incorrect Pattern Dimensions** (60% of cases)
   ```
   Most common: Printed at wrong scale

   Examples:
   - Intended 24mm, actually printed 25mm (4% error)
   - Intended 24mm, actually printed 23mm (4% error)
   - Scale error compounds calibration error

   4% dimension error = ~4% RMS error increase
   ```

2. **Poor Pattern Printing Quality** (20% of cases)
   ```
   - Blurry edges (pixelation)
   - Uneven color (faded printing)
   - Pattern not flat (warped paper)
   - Pattern worn or aged
   ```

3. **Capture Quality Issues** (15% of cases)
   ```
   - Blurry captures (focus issues)
   - Motion blur (pattern moved during exposure)
   - Shadows or uneven lighting
   ```

4. **Camera Calibration Drift** (5% of cases)
   ```
   - Lens focus changed between captures
   - Camera parameters shifted
   - Temperature-induced optical changes
   ```

**Diagnosis Steps:**

**Step 1: Verify Pattern Dimensions**
```
CRITICAL - Do this BEFORE recalibrating:

1. Obtain printed pattern
2. Use digital calipers (±0.1mm precision)
3. Measure 5 different squares:
   - Top-left square
   - Top-right square
   - Center square
   - Bottom-left square
   - Bottom-right square

4. Record measurements:
   Square 1: [___] mm
   Square 2: [___] mm
   Square 3: [___] mm
   Square 4: [___] mm
   Square 5: [___] mm

   Average: [___] mm
   Tolerance: ±0.5mm (acceptable)
             >±0.5mm (recalibrate with correct size)
```

**Step 2: Assess Pattern Quality**
```
Check these in the captured images:
- Sharp edges (not pixelated)?     [ ] Yes [ ] No
- Even contrast (not faded)?       [ ] Yes [ ] No
- No shadows on pattern?           [ ] Yes [ ] No
- Pattern flat (not warped)?       [ ] Yes [ ] No
- All corners distinct?             [ ] Yes [ ] No

If any NO: Pattern quality is issue
```

**Step 3: Review Capture Images**
```
Look at dataset images:
/unlook_calib_dataset/dataset_XXX/left/
/unlook_calib_dataset/dataset_XXX/right/

Check for:
- Blurriness (focus issue)
- Motion blur (camera moved)
- Uneven lighting
- Pattern partially cut off
```

**Solutions:**

**Solution 1: Recalibrate with Correct Pattern Dimensions** (Most Common Fix)
```
If measured square size != 24mm:
1. Measure actual square size with calipers
2. In next calibration session:
   - Set "Square Size" to measured value
   - Example: If actually 25mm, enter 25.0
   - This corrects the dimension error
3. Recapture dataset with corrected setting
4. RMS error should improve significantly
```

**Solution 2: Improve Pattern Quality**
- Print new pattern on better printer
- Use high-quality glossy paper or rigid backing
- Ensure pattern is perfectly flat
- Check contrast is high (black is dark, white is bright)
- Mount on foam board or plexiglass

**Solution 3: Improve Capture Quality**
- Focus cameras better on pattern
- Ensure consistent focus throughout capture
- Improve lighting (softer, more even)
- Avoid motion or vibration
- Use stabilized mounting for pattern

**Solution 4: Check Camera Optics**
- Clean camera lenses with soft cloth
- Check for dust inside lens
- Ensure no damage to optical elements
- If lens damaged, may need replacement

**Verification After Recalibration:**
```
New calibration RMS error should be:
- If dimension was wrong:    0.3-0.5 px (fixed!)
- If pattern quality issue:   0.2-0.4 px (improved)
- If capture issue:           0.2-0.3 px (much better)

If still high (>0.6 px):
- Repeat steps above
- Check for camera lens issues
- Consider professional calibration service
```

---

### High Baseline Error

#### Symptom: Processing shows baseline error > 1.0 mm or "CRITICAL" status

**What this means:**
- Measured baseline (70.12mm) differs significantly from expected (70.00mm)
- Indicates mechanical misalignment or incorrect pattern dimensions
- Critical for depth accuracy - MUST be corrected

**Baseline Severity Scale:**
```
Baseline error (measured vs expected 70mm):

0.0-0.2 mm:   EXCELLENT (±0.3%)
0.2-0.5 mm:   VERY GOOD (±0.7%) <- TARGET
0.5-1.0 mm:   ACCEPTABLE (±1.4%) <- WARNING
1.0-2.0 mm:   POOR (±2.9%) <- NEEDS CORRECTION
>2.0 mm:      CRITICAL (±2.9%+) <- MUST FIX
```

**Root Causes (in order of probability):**

1. **Incorrect Pattern Dimensions** (40% of cases)
   ```
   Pattern dimension error directly causes baseline error:

   Example:
   - Pattern printed 25mm instead of 24mm
   - Calibration assumes 24mm
   - Scales baseline incorrectly by 25/24 = 4.2% error
   - Expected 70mm, measures ~73mm error!

   Dimension error % = Baseline error %
   ```

2. **Mechanical Misalignment** (35% of cases)
   ```
   Camera physical baseline changed:

   Causes:
   - Mounting plate bent/flexed
   - Screws loosened
   - Thermal expansion/contraction
   - Camera shifted in mount
   - Impact or collision

   Check:
   - Measure baseline with calipers?
   - Are all mounting screws tight?
   - Is mounting plate flat?
   - Any visible damage?
   ```

3. **Calibration Algorithm Issue** (15% of cases)
   ```
   Poor calibration quality:
   - Insufficient valid image pairs
   - Poor image variety
   - Calibration algorithm convergence

   Check:
   - Valid image pairs >= 30?
   - RMS error reasonable?
   - Epipolar error acceptable?
   ```

4. **Temperature-Induced Drift** (10% of cases)
   ```
   Optical component expansion:
   - Temperature change >20°C
   - Thermal equilibrium not reached
   - Baseline shifts slightly

   Solution: Allow warm-up before calibration
   ```

**Diagnosis Steps:**

**Step 1: Verify Pattern Dimensions (FIRST - Most Common Cause)**
```
This is the most common cause of baseline error!

1. Get printed pattern
2. Use digital calipers (±0.1mm precision)
3. Measure multiple squares as before
4. Calculate average square size

If average != 24.0 ± 0.5 mm:
   This WILL cause baseline error
   Recalibrate with CORRECT dimension setting
```

**Step 2: Measure Physical Baseline**
```
If pattern dimensions are correct, check mechanical baseline:

1. Turn off cameras
2. Measure distance between camera centers
   - Use digital calipers or ruler
   - Measure precisely (mm precision)
   - Expected: 70.0 mm

3. If measured != 70mm:
   - Physical baseline has shifted
   - Check mechanical mounting
   - Look for bent plates or loose screws
```

**Step 3: Check Mechanical Mounting**
```
Inspect camera mounting:

[ ] All mounting screws are tight
    - Check each screw with screwdriver
    - Ensure firm (not over-tight)
    - Check mounting plate for bend

[ ] Mounting plate is flat
    - Visual inspection for bends
    - Check with straightedge
    - No flex when gently pressed

[ ] Cameras are securely seated
    - No gaps between camera and mount
    - Both cameras same height
    - No lateral shift

[ ] No visible damage
    - No dents or cracks
    - No loose connectors
    - All cables secure

If any defect found:
   - Loosen mounting screws
   - Realign cameras
   - Re-tighten screws gradually
   - Re-measure baseline
```

**Step 4: Temperature Check**
```
Has temperature changed recently?

Recent large temperature change?
- Allow 30+ minutes warm-up
- Thermal equilibrium needed
- Try calibration again after warming up
```

**Solutions:**

**Solution 1: Recalibrate with Correct Pattern Dimensions** (Most Likely Fix)

**This is THE most common fix for baseline error.**

```cpp
If pattern square size measured wrong:

Before: Entered 24.0 mm (incorrect)
Actual: Pattern printed at 25.2 mm (measured with calipers)
Error: 25.2/24.0 = 5% dimension error
       = 5% baseline error (70mm -> 73.5mm)

Fix:
1. Update pattern dimension in UI
   Square Size: [ENTER ACTUAL MEASURED VALUE]

2. Recapture dataset (don't reuse old dataset)
3. Process new dataset
4. Baseline should now be ~70.0 mm

Expected result:
   Baseline error drops from ~3.5mm to <0.5mm
```

**Solution 2: Fix Mechanical Mounting**

If baseline dimension is correct but baseline error still high:

```
1. Loosen all camera mounting screws
   - Remove each screw slightly
   - Don't remove completely

2. Re-align cameras
   - Check baseline with calipers
   - Ensure cameras at same height
   - Ensure baseline = 70.0 ± 0.5 mm

3. Re-tighten screws gradually
   - Tighten each screw partially
   - Then tighten all fully
   - Check baseline hasn't shifted

4. Recalibrate
   - Capture new dataset
   - Baseline error should be <0.5 mm now
```

**Solution 3: Re-establish Thermal Equilibrium**

If recent large temperature change:

```
1. Allow scanner 30+ minutes warm-up
2. Room temperature should be stable
3. Thermal drift will stop after equilibrium
4. Recalibrate after warming up

This fixes temperature-induced baseline shift
```

**Solution 4: Full Recalibration with Better Data**

If mechanical and dimensional checks pass:

```
Recapture with focus on quality:
- More varied positions (45+ valid pairs ideal)
- Better angle variety
- Better position coverage
- Improved lighting consistency

Better calibration data -> Better baseline accuracy
```

**Critical Warning:**

```
DO NOT USE CALIBRATION WITH BASELINE ERROR > 1.0 MM
FOR PRODUCTION SCANNING!

High baseline error causes:
- Depth measurement errors proportional to error %
- 1mm baseline error = ~1mm depth error per meter distance
- 3D models will have systematic scale error

ACTION REQUIRED:
1. Find root cause using steps above
2. Correct mechanical or dimensional issue
3. Recalibrate
4. Verify baseline error <0.5 mm before use
```

---

### Processing Failures and Crashes

#### Symptom: Processing hangs, crashes, or shows error message

**What this means:**
- Processing algorithm encountered error or ran out of resources
- Could be out of memory, corrupted images, or invalid dataset

**Common Error Messages:**

```
Error: "Insufficient valid image pairs: 18 (need >= 30)"
Action: See "Low Valid Pairs" section above

Error: "OpenCV error: calibration failed"
Cause: Severe calibration problem (numerical issues)
Action: Recapture with better data quality

Error: "Cannot load dataset JSON"
Cause: Corrupted dataset_info.json
Action: Check dataset folder contents

Error: "Out of memory"
Cause: Insufficient RAM
Action: Close other applications, increase available memory
```

**Diagnosis Steps:**

**Step 1: Check System Resources**
```bash
# Check free memory
free -h

# Check disk space
df -h /unlook_calib_dataset
df -h /unlook_calib

# Check running processes
ps aux | grep unlook

# If memory < 100MB:
   - Close other applications
   - Reduce number of parallel processes
   - Increase available RAM
```

**Step 2: Check Dataset Integrity**
```bash
# Verify dataset directory structure
ls -la /unlook_calib_dataset/dataset_YYYYMMDD_HHMMSS/
# Should see: left/, right/, dataset_info.json

# Check PNG files are valid
file /unlook_calib_dataset/dataset_YYYYMMDD_HHMMSS/left/frame_*.png
# Should show "PNG image data"

# Verify JSON is valid
python3 -m json.tool /unlook_calib_dataset/dataset_YYYYMMDD_HHMMSS/dataset_info.json
# Should show valid JSON structure
```

**Step 3: Check Logs**
```bash
# View calibration processing logs
tail -100 ~/.unlook/logs/calibration.log

# Look for specific error messages
grep -i "error" ~/.unlook/logs/calibration.log
grep -i "exception" ~/.unlook/logs/calibration.log
```

**Solutions:**

**Solution 1: Free System Resources**
```bash
# Close memory-heavy applications
pkill -f firefox
pkill -f chromium
pkill -f blender

# Clear temporary files
rm -rf /tmp/*

# Reduce background processes
systemctl stop bluetooth (if running)
systemctl stop cups (if running)
```

**Solution 2: Restart Application**
```bash
# Kill running unlook process
pkill -f unlook

# Wait 5 seconds
sleep 5

# Restart
unlook
```

**Solution 3: Verify and Repair Dataset**
```bash
# Check dataset contents
cd /unlook_calib_dataset/dataset_YYYYMMDD_HHMMSS/

# Count PNG files
ls left/frame_*.png | wc -l  # Should be 50
ls right/frame_*.png | wc -l # Should be 50

# Check for corrupted images
file left/frame_*.png | grep -v "PNG image"
# Should have no output (all valid PNGs)

# Validate JSON
python3 -c "import json; json.load(open('dataset_info.json'))"
# Should show no error
```

**Solution 4: Start Fresh Capture**
```
If dataset is corrupted:
1. Delete dataset folder
2. Start new capture session
3. Recapture 50 image pairs
4. Process new dataset
```

---

### Calibration Not Applied to System

#### Symptom: Depth measurements still wrong after "successful" calibration

**What this means:**
- Calibration file was created but not set as system default
- OR old calibration is still active
- OR incorrect calibration file path

**Diagnosis Steps:**

**Step 1: Check System Default**
```bash
# Check what's the default calibration
ls -la /unlook_calib/default.yaml

# Should show symlink to latest calibration
# Example: default.yaml -> calib-20250104_153000.yaml

# If symlink doesn't exist or points to old file:
   - System is not using new calibration
```

**Step 2: Verify Latest Calibration Exists**
```bash
# List all calibration files
ls -la /unlook_calib/calib-*.yaml

# Check timestamp of latest
ls -lt /unlook_calib/calib-*.yaml | head -1

# Should show recent timestamp (matching calibration session)
```

**Step 3: Check Calibration File Contents**
```bash
# View calibration file
head -20 /unlook_calib/calib-YYYYMMDD_HHMMSS.yaml

# Check for these fields:
# - calibration_date
# - camera_matrix_left
# - camera_matrix_right
# - translation_vector (baseline)

# Missing fields = invalid file
```

**Step 4: Verify Permissions**
```bash
# Check read permissions
ls -la /unlook_calib/
# Should show rwx for calibration directory

# Check file permissions
ls -la /unlook_calib/default.yaml
# Should be readable
```

**Solutions:**

**Solution 1: Manually Set Default Calibration**
```bash
# Find latest calibration
ls -lt /unlook_calib/calib-*.yaml | head -1
# Note the filename: calib-YYYYMMDD_HHMMSS.yaml

# Remove old symlink
rm /unlook_calib/default.yaml

# Create new symlink
ln -s /unlook_calib/calib-YYYYMMDD_HHMMSS.yaml /unlook_calib/default.yaml

# Verify
ls -la /unlook_calib/default.yaml
# Should show symlink to latest calibration

# Restart application
pkill unlook
sleep 2
unlook
```

**Solution 2: Recalibrate with Auto-Default**
```
1. Perform complete calibration workflow
2. System should auto-update /unlook_calib/default.yaml
3. Verify with: ls -la /unlook_calib/default.yaml
4. Restart application
5. Test depth measurements
```

**Solution 3: Clear Old Calibrations**
```bash
# Backup current calibration
cp /unlook_calib/default.yaml /unlook_calib/default.yaml.bak

# Remove old calibration files
rm /unlook_calib/calib-*.yaml

# Perform new calibration
# This ensures only new calibration is present
```

---

### Verification and Testing

#### After calibration completes, verify it's working:

**Test 1: Visual Inspection**
```
1. Open Depth Test tab
2. Scan a known object (cube, sphere)
3. Visual check: 3D point cloud should be:
   - No scattered outliers
   - Clean surface representation
   - Correct overall shape and size
```

**Test 2: Measurement Verification**
```
1. Measure known object (e.g., 10cm cube)
2. Scan with Unlook
3. Measure scanned result
4. Compare dimensions
5. Accuracy should be within 5-10mm (0.5-1% error)
```

**Test 3: Check Calibration Parameters**
```bash
# View calibration baseline
grep "tx:" /unlook_calib/default.yaml
# Should show approximately -70.0 mm

# View camera focal lengths
grep "fx:" /unlook_calib/default.yaml
grep "fy:" /unlook_calib/default.yaml
# Should be approximately 1200-1250 pixels
```

**Test 4: Check Quality Metrics**
```bash
# View RMS error
grep "rms_reprojection_error:" /unlook_calib/default.yaml
# Should be < 0.3 pixels

# View baseline error
grep "baseline_error_mm:" /unlook_calib/default.yaml
# Should be < 0.5 mm
```

**If tests fail:**
- See specific issue sections above
- Recalibrate with attention to detailed steps
- Consult detailed troubleshooting guides

---

## Error Message Reference

### During Capture

| Message | Cause | Solution |
|---------|-------|----------|
| Pattern Not Detected | Focus/lighting issue | Reposition, check focus, improve light |
| ✗ Detected: 0 corners | Pattern not visible | Ensure pattern is in frame |
| ✗ Detected: <50 corners | Partial detection | Improve angle, focus, or lighting |

### During Processing

| Message | Cause | Solution |
|---------|-------|----------|
| Insufficient valid pairs | Low detection rate | Recapture with better quality |
| OpenCV calibration failed | Severe issue | Check pattern dimensions |
| Cannot read dataset JSON | Corrupted file | Recapture dataset |
| Out of memory | Insufficient RAM | Close other apps, increase RAM |

### Quality Metrics

| Metric | Issue | Cause | Solution |
|--------|-------|-------|----------|
| RMS > 0.6px | Poor accuracy | Bad pattern/dimensions | Verify dimensions, recapture |
| Baseline > 1mm | Mechanical issue | Mount problem/wrong dimensions | Check mount, verify dimensions |
| Epipolar > 1.0px | Bad rectification | Calibration issue | Improve data quality |
| Valid < 30 | Detection failure | Focus/lighting | Better capture technique |

---

## Prevention Tips

### Before Every Calibration Session

- [ ] Verify pattern dimensions with calipers (24.0 ± 0.5mm)
- [ ] Check camera lenses are clean
- [ ] Ensure cameras are rigidly mounted
- [ ] Allow warm-up period (30+ minutes)
- [ ] Test lighting conditions
- [ ] Free up system memory
- [ ] Have backup power available

### Best Practices

- [ ] Always measure pattern with calipers
- [ ] Do test detection before starting capture
- [ ] Vary positions and angles systematically
- [ ] Monitor for failed detections during capture
- [ ] Keep detailed notes of calibration sessions
- [ ] Backup old calibrations before new ones
- [ ] Test on known objects after calibration
- [ ] Document calibration date and conditions

---

## When to Contact Support

Contact manufacturer support if:

1. **Repeated calibration failures** despite following all steps
2. **Consistent high RMS error** (> 1.0 px) on multiple attempts
3. **Baseline error > 2.0mm** on multiple attempts
4. **Processing crashes** on multiple datasets
5. **Hardware appears damaged** (bent mounting, loose connectors)
6. **Depth accuracy not improving** after successful recalibration

**Provide to support:**
- Most recent calibration YAML file
- System information (OS, RAM, storage)
- Screenshots of quality metrics
- List of troubleshooting steps attempted
- Details of pattern (dimensions, condition)
- Photos of camera mounting setup

---

## Summary

### Quick Reference Flowchart

```
START: Calibration Issue
│
├─ Pattern not detected?
│  └─> Check focus, lighting, positioning
│       -> Reposition pattern
│       -> Improve lighting
│       -> Verify pattern configuration
│
├─ Low valid pairs (<30)?
│  └─> Check capture technique
│       -> Recapture with better variety
│       -> Ensure pattern stays in focus
│       -> Monitor detection during capture
│
├─ RMS error high (>0.6px)?
│  └─> FIRST: Verify pattern dimensions with calipers!
│       -> If wrong: Recalibrate with correct dimensions
│       -> If correct: Check pattern quality
│       -> Improve capture conditions
│
├─ Baseline error high (>1mm)?
│  └─> FIRST: Verify pattern dimensions with calipers!
│       -> If wrong: Recalibrate with correct dimensions
│       -> If correct: Check mechanical mounting
│       -> Verify physical baseline with calipers
│       -> Re-tighten mounting screws
│
├─ Processing hangs/crashes?
│  └─> Free system memory
│       -> Delete corrupted dataset
│       -> Start fresh capture
│       -> Restart application
│
└─ After successful calibration, depth still wrong?
   └─> Check system default symlink
        -> Manually set default if needed
        -> Restart application
        -> Test on known object
```

### Most Common Issues and Quick Fixes

**1. Pattern not detected (40% of cases)**
- Solution: Improve focus and lighting
- Time to fix: 2-5 minutes

**2. Incorrect pattern dimensions (35% of cases)**
- Solution: Verify with calipers, recalibrate with correct size
- Time to fix: 10-15 minutes

**3. Low valid pairs (15% of cases)**
- Solution: Recapture with better positioning variety
- Time to fix: 5-10 minutes

**4. Mechanical misalignment (10% of cases)**
- Solution: Check and re-tighten camera mounting
- Time to fix: 5-10 minutes

---

**Document Version**: 1.0
**Last Updated**: 2025-01-04
**Status**: Production Ready
