# Stereo Calibration Parameters Reference

## Table of Contents
1. Pattern Configuration
2. Camera Intrinsics
3. Stereo Extrinsics
4. Rectification Parameters
5. Quality Metrics
6. Configuration Examples

---

## Pattern Configuration

The calibration process begins by defining the target pattern used for camera pose estimation.

### PatternType Enumeration

```cpp
enum class PatternType {
    CHECKERBOARD,   // Classic black/white checkerboard
    CHARUCO,        // ArUco markers + checkerboard (RECOMMENDED)
    CIRCLE_GRID     // Asymmetric circle pattern
};
```

### ChArUco Pattern (Recommended)

**Why ChArUco?**
- Robust to partial occlusion (some markers can be hidden)
- Better detection with infrared/VCSEL illumination
- Resistant to lighting variations
- Faster detection algorithm
- Subpixel corner accuracy

**Configuration Parameters:**

```yaml
pattern_config:
  type: "charuco"                    # Pattern type identifier
  rows: 7                            # Vertical pattern count
  cols: 10                           # Horizontal pattern count
  square_size_mm: 24.0               # Physical square dimension (CRITICAL!)
  aruco_marker_size_mm: 17.0         # ArUco marker dimension
  aruco_dictionary: "DICT_4X4_250"  # ArUco marker dictionary
```

**Parameter Details:**

### rows (Integer: 4-20)
- **Meaning**: Number of square rows in the checkerboard pattern
- **Typical Value**: 7
- **Constraints**: Must match printed pattern exactly
- **Effect**: Affects number of calibration points per image (rows × cols)
- **Minimum**: 4×4 = 16 points per image
- **Recommended**: 7×10 = 70 points per image

**Example:**
```
Standard ChArUco 7×10 pattern:
7 rows (vertical)
10 columns (horizontal)
Total 70 detection points per image pair
```

### cols (Integer: 4-20)
- **Meaning**: Number of square columns in the checkerboard pattern
- **Typical Value**: 10
- **Constraints**: Must match printed pattern exactly
- **Effect**: See rows above
- **Note**: More columns (10) than rows (7) provides better horizontal coverage

### square_size_mm (Float: 5.0-100.0)
- **Meaning**: Physical dimension of each checkerboard square
- **Typical Value**: 24.0 mm
- **Unit**: Millimeters
- **CRITICAL**: Must be measured with calipers and entered EXACTLY
- **Tolerance**: ±0.5 mm acceptable, >±0.5 mm will cause errors
- **Effect**: 1mm error in square size = 1mm error in baseline

**Why this matters:**
```cpp
// Calibration formula (simplified)
baseline_mm = T[0] * scale_factor

// If square size wrong:
expected_square = 24.0 mm
actual_square = 25.0 mm      // Printed wrong or entered wrong
scale_factor = actual / expected = 25.0 / 24.0 = 1.042

// Baseline calculation gets scaled wrong:
true_baseline = 70.0 mm
calculated_baseline = 70.0 * 1.042 = 72.94 mm
ERROR = 2.94 mm !!! (4% error)
```

**Verification Procedure:**
```
1. Use digital calipers (±0.1mm precision)
2. Measure 5 squares across pattern
3. Record each measurement
4. Calculate average
5. If average != 24.0 ± 0.5 mm:
   STOP - print new pattern or enter correct measured value
```

### aruco_marker_size_mm (Float: 5.0-50.0)
- **Meaning**: Physical size of ArUco markers embedded in pattern
- **Typical Value**: 17.0 mm
- **Note**: Must be smaller than square size
- **Relationship**: aruco_size < square_size (e.g., 17 < 24)
- **Effect**: Affects ArUco detection robustness

**Typical Relationships:**
```
Square Size: 24mm -> Marker Size: 17mm (70.8% of square)
Square Size: 25mm -> Marker Size: 17.5mm (70% of square)
Square Size: 20mm -> Marker Size: 14mm (70% of square)

Rule: Marker size ≈ 70% of square size
```

### aruco_dictionary (String)
- **Meaning**: ArUco marker dictionary ID
- **Value**: "DICT_4X4_250"
- **Meaning**: 4×4 bit markers, 250 unique markers
- **Why this value**: Good balance of distinctiveness and robustness
- **Do not change**: Must match markers printed on pattern

**Common Dictionaries:**
```
DICT_4X4_50    - 4×4 bits, 50 markers, faster detection
DICT_4X4_100   - 4×4 bits, 100 markers
DICT_4X4_250   - 4×4 bits, 250 markers (RECOMMENDED)
DICT_4X4_1000  - 4×4 bits, 1000 markers, slower
DICT_5X5_100   - 5×5 bits, 100 markers, more robust
DICT_5X5_250   - 5×5 bits, 250 markers
DICT_6X6_250   - 6×6 bits, 250 markers, slowest
```

---

## Camera Intrinsics

Camera intrinsic parameters describe how each individual camera projects 3D world points onto the 2D image plane.

### Camera Matrix Format

```yaml
camera_matrix_left:
  fx: 1220.5          # Focal length (x-axis) in pixels
  fy: 1221.2          # Focal length (y-axis) in pixels
  cx: 640.1           # Principal point x-coordinate
  cy: 360.3           # Principal point y-coordinate
```

**Matrix Representation:**
```
K = [fx  0  cx]
    [ 0 fy  cy]
    [ 0  0   1]

Where:
- fx, fy: focal lengths in pixels
- cx, cy: principal point (image center)
```

### fx, fy (Focal Length in Pixels)

- **Meaning**: Distance from camera center to image plane
- **Typical Value**: 1200-1250 pixels (for IMX296 with 6mm lens)
- **Unit**: Pixels
- **Why pixels?**: Depends on sensor resolution and lens
- **Relationship to physical focal length**:
  ```
  fx_pixels = f_mm * image_width / sensor_width

  Example for IMX296:
  - Physical focal length: f = 6.0 mm
  - Sensor width: ~5.6 mm (1456 pixels)
  - f_x = 6.0 * 1456 / 5.6 ≈ 1560 pixels

  Note: After downsampling to HD 1280×720:
  f_x ≈ 1560 * (1280/1456) ≈ 1372 pixels
  ```

- **Effect**: Affects depth measurement scale
- **Error**: 1% error in focal length = 1% error in depth

### cx, cy (Principal Point)

- **Meaning**: Pixel coordinates of camera optical axis intersection
- **Typical Value**: (640, 360) for HD 1280×720 resolution
- **Note**: Usually close to image center, but not always exactly centered
- **Typical Offset**: ±5-10 pixels from center (due to lens distortion)

**Example:**
```
HD Resolution: 1280×720 pixels
Expected center: (640, 360)
Actual principal point: (640.1, 360.3)
Deviation: 0.1 pixels (negligible)

If deviation > 5 pixels:
- Check for lens mounting tilt
- May indicate optical alignment issue
```

### Focal Length Relationship

```
fx and fy should be very similar:

OK: fx=1220.5, fy=1221.2  (difference: 0.7 pixels, 0.06%)
WARNING: fx=1220, fy=1250  (difference: 30 pixels, 2.5%)
FAIL: fx=1000, fy=1500     (difference: 500 pixels)

Large difference suggests:
- Lens mounting tilt
- Sensor mounting tilt
- Optical element damage
```

### Distortion Coefficients

```yaml
distortion_coeffs_left:
  k1: -0.15           # Radial distortion 1st order
  k2: 0.08            # Radial distortion 2nd order
  p1: 0.001           # Tangential distortion x
  p2: -0.002          # Tangential distortion y
  k3: -0.02           # Radial distortion 3rd order (Brown model)
```

**Distortion Model (Brown-Conrady):**
```
Undistorted coordinates from distorted:

x_undist = x_dist * (1 + k1*r² + k2*r⁴ + k3*r⁶) +
           (2*p1*x*y + p2*(r² + 2*x²))

y_undist = y_dist * (1 + k1*r² + k2*r⁴ + k3*r⁶) +
           (p1*(r² + 2*y²) + 2*p2*x*y)

where r² = (x - cx)² + (y - cy)²
```

**Coefficient Interpretation:**

### k1 (Radial Distortion, 1st Order)
- **Range**: Typically -0.3 to +0.3
- **Meaning**:
  - Negative: Barrel distortion (straight lines bend outward)
  - Positive: Pincushion distortion (straight lines bend inward)
- **Typical for 6mm lens**: -0.10 to -0.20 (slight barrel)
- **Example**: k1 = -0.15 means moderate barrel distortion

### k2 (Radial Distortion, 2nd Order)
- **Range**: Typically -0.1 to +0.2
- **Meaning**: Corrects higher-order radial distortion
- **Typical for IMX296**: +0.05 to +0.12
- **Usually positive**: Compensates for k1 nonlinearity

### p1, p2 (Tangential/Decentering Distortion)
- **Range**: Typically ±0.001 to ±0.01
- **Meaning**: Distortion due to lens mounting tilt
- **Typical for IMX296**: Usually very small (<±0.005)
- **Large values** (>±0.01): Indicate lens mount alignment issue

### k3 (Radial Distortion, 3rd Order)
- **Range**: Typically -0.05 to +0.05
- **Meaning**: Fine correction for extreme distortion
- **Typical for IMX296 with 6mm**: -0.02 to 0.0
- **Often zero**: Many cameras don't need 3rd order correction

**Distortion Check:**
```
Well-calibrated camera:
k1: -0.15       (moderate barrel, normal for 6mm)
k2: +0.08       (compensates k1)
p1, p2: ±0.001  (very small, good alignment)
k3: -0.02       (small, typical)

Suspect calibration:
k1: -0.50       (excessive barrel, check lens)
p1, p2: ±0.05   (large, check lens mount alignment)
k3: ±0.10+      (excessive, may indicate optical issue)
```

---

## Stereo Extrinsics

Stereo extrinsics describe the relative position and orientation between the two cameras.

### Rotation Matrix (3×3)

```yaml
rotation_matrix:
  - [0.9998, -0.0001, 0.0201]
  - [0.0001,  1.0000, 0.0003]
  - [-0.0201, -0.0003, 0.9998]
```

**Mathematical Meaning:**
```
R: Rotation from left camera frame to right camera frame
R = [r11 r12 r13]
    [r21 r22 r23]
    [r31 r32 r33]

For stereo pair with minimal baseline rotation:
R ≈ Identity matrix [1 0 0; 0 1 0; 0 0 1]

In example above:
- Main diagonal near 1: Good (minimal rotation)
- Off-diagonal small: Good (< 0.02)
- Determinant = 1: Orthogonal (good)
```

**Interpretation:**
```
Perfect horizontal baseline:
R = [1  0  0]
    [0  1  0]
    [0  0  1]

Slight tilt in our example:
- r13 = 0.0201 (2cm tilt rotation, <2 degrees)
- r31 = -0.0201 (compensating tilt)
- Indicates very slight optical axis tilt
- Negligible effect on calibration (< 0.1 degree)
```

**Check for Issues:**
```
Well-aligned stereo pair:
- Main diagonal values > 0.99
- Off-diagonal values < ±0.05
- Rotation angle < 2 degrees

Misaligned stereo pair:
- Main diagonal < 0.98
- Off-diagonal > ±0.1
- Rotation angle > 5 degrees
- Indicates mechanical misalignment
```

### Translation Vector (3D)

```yaml
translation_vector:
  tx: -70.12          # X-component (BASELINE!) in mm
  ty: 0.15            # Y-component in mm
  tz: -0.08           # Z-component in mm
```

**Mathematical Meaning:**
```
T: Translation from left camera to right camera
T = [tx, ty, tz]'

In 4×4 extrinsic matrix:
[R11 R12 R13 tx]
[R21 R22 R23 ty]
[R31 R32 R33 tz]
[0   0   0   1 ]

Point transformation:
P_right = R * P_left + T
```

### tx (Baseline in X, CRITICAL)

- **Meaning**: Horizontal distance between cameras
- **Expected Value**: -70.0 mm (camera 1 is 70mm to the left of camera 0)
- **Negative value**: Means right camera is ~70mm to the right
- **Physical baseline**: ~70mm (design specification)
- **Tolerance**: ±0.5 mm acceptable, ±1.0 mm warning
- **Critical for**: Depth scale accuracy

**Why negative?**
```
Camera coordinate frame:
- X: rightward
- Y: downward
- Z: forward

Right camera relative to left:
- 70mm to the RIGHT in world coordinates
- -70mm in left camera's coordinate frame (negative X)

tx = -70.12 mm is CORRECT
```

**Baseline Error Impact:**
```
Baseline affects depth scale:

Baseline error = |measured - expected| = |70.12 - 70.0| = 0.12mm
Error percentage = 0.12 / 70.0 = 0.17%

Depth measurement error scales proportionally:
1-meter distance: depth error ≈ 1.7mm
2-meter distance: depth error ≈ 3.4mm

0.5mm baseline error: depth error ~7mm at 1m
1.0mm baseline error: depth error ~14mm at 1m (unacceptable)
```

### ty (Baseline in Y, Vertical)

- **Meaning**: Vertical offset between cameras
- **Expected Value**: ~0.0 mm (cameras aligned horizontally)
- **Typical Value**: ±0.2 mm (minor alignment error)
- **Warning**: > ±0.5 mm indicates vertical misalignment
- **Effect**: Causes vertical disparity mismatch

### tz (Baseline in Z, Depth)

- **Meaning**: Depth offset between cameras
- **Expected Value**: ~0.0 mm (cameras at same depth)
- **Typical Value**: ±0.1 mm
- **Warning**: > ±0.5 mm indicates depth offset
- **Effect**: Causes convergence error

**Combined Baseline Magnitude:**
```
|T| = sqrt(tx² + ty² + tz²)
    = sqrt((-70.12)² + (0.15)² + (-0.08)²)
    = sqrt(4916.8 + 0.02 + 0.006)
    = sqrt(4916.83)
    = 70.12 mm

Very close to expected 70.0 mm (0.17% error) -> GOOD!
```

---

## Rectification Parameters

Rectification transforms the image pair so that corresponding points lie on the same horizontal scanline.

### Rectification Transform Matrices (R1, R2)

```yaml
rectification_transform_left: 3x3 matrix
rectification_transform_right: 3x3 matrix
```

**Purpose:**
```
After rectification:
- Corresponding features have same Y coordinate
- Disparity is purely horizontal (X direction)
- Stereo matching becomes 1D search along scanline

Before rectification:
- Corresponding features at different Y
- Must search 2D area around match point
- Much slower and less accurate

Rectified stereo enables:
- Efficient block matching algorithms
- Reliable depth computation
- Real-time processing capability
```

**Mathematical Properties:**
```
R1, R2 are orthogonal rotation matrices (3×3)
R1' * R1 = I  (R1 is its own inverse)
det(R1) = 1   (determinant is 1)

Applied as:
P_left_rectified = R1 * P_left
P_right_rectified = R2 * P_right
```

**Effect on Image Coordinates:**
```
Original image points:
p_left = [x_l, y_l]'
p_right = [x_r, y_r]'

After rectification:
p_left_rect = R1 * p_left
p_right_rect = R2 * p_right

Property: y_left_rect == y_right_rect (same Y after rectification)
```

### Projection Matrices (P1, P2)

```yaml
projection_matrix_left: 3x4 matrix
projection_matrix_right: 3x4 matrix
```

**Format:**
```
P1 = K1 * [R1 | t1]  (3×4 matrix)
P2 = K2 * [R2 | t2]  (3×4 matrix)

Where:
K: Camera intrinsic matrix (3×3)
R: Rectification rotation (3×3)
t: Translation component (3×1)

Result:
[x]     [fx  0 cx  0]   [X]
[y] = [ 0 fy cy  0] * [Y]
[1]     [ 0  0  1  0]   [Z]
                        [1]
```

**Projection Matrix Effect:**
```
Projects 3D point P_world to 2D image coordinates:

p_image = P * P_world / Z

P1 projects to left image
P2 projects to right image

After rectification:
- Both have same Y values (canonical form)
- Disparity computed as: d = x_left - x_right
```

### Disparity-to-Depth Matrix (Q)

```yaml
disparity_to_depth_matrix: 4x4 matrix
```

**Purpose:** Converts disparity map to depth map

**Format:**
```
Q = [1  0  0  -cx]
    [0  1  0  -cy]
    [0  0  0   f ]
    [0  0 -1/B  0]

Where:
cx, cy: Principal point
f: Focal length
B: Baseline (70mm)

Usage to convert disparity to depth:
[X]       [x]
[Y] = Q * [y]
[Z]       [d] (disparity)
[W]       [1]

X = x * Z / f
Y = y * Z / f
Z = f * B / d

where d = disparity (x_left - x_right)
```

**Depth Calculation Example:**
```
Stereo pair with:
- Baseline B = 70mm
- Focal length f = 1220 pixels
- Disparity d = 40 pixels

Depth Z = f * B / d
        = 1220 * 70 / 40
        = 2135 mm
        = 2.135 meters

Disparity-depth relationship:
- Larger disparity (d) = closer object (smaller Z)
- Smaller disparity (d) = farther object (larger Z)
- Zero disparity = infinite distance (parallel rays)
```

### Rectification Maps

```yaml
rectification_map_left_x: "calib-TIMESTAMP-map-left-x.bin"
rectification_map_left_y: "calib-TIMESTAMP-map-left-y.bin"
rectification_map_right_x: "calib-TIMESTAMP-map-right-x.bin"
rectification_map_right_y: "calib-TIMESTAMP-map-right-y.bin"
```

**Purpose:**
```
Maps store per-pixel remapping coordinates
Used for efficient image rectification:

cv::remap(input_image,
          output_image,
          map_x, map_y,
          cv::INTER_LINEAR)

Instead of computing transform for every pixel,
lookup tables provide O(1) rectification
```

**Format:**
```
Each map is 1280×720 (HD resolution)
- Float32 values for each pixel
- Stores (x, y) coordinate in original image
- Used to pull pixels from original image

map_x[y][x] = source_x coordinate in original image
map_y[y][x] = source_y coordinate in original image
```

**Binary Storage:**
```
File format: IEEE 32-bit floating point binary
Size: 1280 * 720 * 4 bytes = 3.68 MB per map
Total for 4 maps: ~14.7 MB

Loading in C++:
cv::FileStorage fs("map-left-x.bin", cv::FileStorage::READ);
cv::Mat mapLeftX;
fs["map"] >> mapLeftX;
fs.release();
```

---

## Quality Metrics

Quality metrics evaluate calibration accuracy and reliability.

### RMS Reprojection Error

```yaml
rms_reprojection_error: 0.28  # pixels
mean_reprojection_error_left: 0.26
mean_reprojection_error_right: 0.29
max_reprojection_error: 0.87
```

**Meaning:**
```
Measures how well detected corners match the calibration model

Error = sqrt(sum((projected - detected)²) / N)

Where:
- projected: Corner position according to calibration
- detected: Actual detected corner position
- N: Number of measurements
```

**Interpretation:**
```
Ideal value: 0.0 pixels (perfect fit)

0.0-0.2 px:  Excellent (professional quality)
0.2-0.3 px:  Very good (recommended target)
0.3-0.5 px:  Good (acceptable)
0.5-0.8 px:  Fair (warning zone)
0.8-1.2 px:  Poor (recalibrate)
>1.2 px:     Unacceptable (invalid calibration)
```

**What Causes High RMS Error?**
```
1. Incorrect pattern dimensions (40% of cases)
   - Pattern printed at wrong scale
   - Entered wrong square size in UI
   - Fix: Verify with calipers, recalibrate

2. Poor pattern printing quality (20% of cases)
   - Blurry edges, faded printing, warped pattern
   - Fix: Print new pattern on better equipment

3. Capture quality issues (20% of cases)
   - Blurry images, motion blur, poor focus
   - Fix: Improve focus, lighting, stability

4. Algorithmic issues (20% of cases)
   - Corner detection problems, outliers
   - Fix: Improve image variety, more valid pairs
```

**Per-Camera Analysis:**
```
If mean_left and mean_right differ significantly:

mean_left = 0.26 px
mean_right = 0.29 px
Difference: 0.03 px (negligible)

mean_left = 0.15 px
mean_right = 0.45 px
Difference: 0.30 px (significant!)

Large difference suggests:
- One camera has poorer focus
- Different lens quality
- Unequal distortion correction
```

### Epipolar Error

```yaml
mean_epipolar_error: 0.15  # pixels
max_epipolar_error: 0.42
```

**Meaning:**
```
Measures how well stereo rectification works

For a point in left image, its corresponding point
in right image should lie on the same horizontal line
(epipolar constraint)

Error = vertical distance of matched points
        (after rectification, should be 0)
```

**Interpretation:**
```
Ideal value: 0.0 pixels (perfect horizontal epipolar lines)

0.0-0.3 px:  Excellent rectification
0.3-0.5 px:  Very good (recommended)
0.5-1.0 px:  Good (acceptable)
1.0-2.0 px:  Fair (may affect stereo matching)
>2.0 px:     Poor (significant matching issues)
```

**Physical Interpretation:**
```
Epipolar error = vertical misalignment between stereo pair

If epipolar error = 0.5 pixels:
- Corresponding points vertically offset by 0.5 pixels
- Block matcher must search vertically around this offset
- Reduces matching confidence and accuracy
- Acceptable but not ideal

If epipolar error = 2.0 pixels:
- Significant vertical offset
- Block matcher may fail to find correspondence
- Stereo matching becomes unreliable
- Indicates poor rectification
```

### Baseline Error

```yaml
baseline_mm: 70.12
baseline_expected_mm: 70.0
baseline_error_mm: 0.12
baseline_error_percent: 0.17
```

**Meaning:**
```
Measures accuracy of measured baseline vs expected

baseline_error_mm = |measured - expected|
baseline_error_percent = (baseline_error_mm / expected) * 100
```

**Interpretation:**
```
0.0-0.2 mm:   Excellent (±0.3%)
0.2-0.5 mm:   Very good (±0.7%, recommended)
0.5-1.0 mm:   Good (±1.4%, acceptable)
1.0-2.0 mm:   Fair (±2-3%, needs attention)
>2.0 mm:      Poor (±3%+, mechanical issue)
```

**Depth Impact:**
```
Baseline error causes proportional depth error:

For object at distance D meters:
Depth error = (baseline_error_mm / baseline_mm) * D * 1000 mm

Examples with B_error = 0.5mm, B = 70mm (0.71% error):

Distance    Depth Error
─────────────────────────
1 meter     ±7 mm
2 meters    ±14 mm
5 meters    ±36 mm
10 meters   ±71 mm
```

---

## Validation Summary

Quality checklist for successful calibration:

```
Quality Metric          Target      Acceptable    Fail If
────────────────────────────────────────────────────────────
RMS Error              <0.3 px     <0.6 px       >0.8 px
Epipolar Error         <0.3 px     <0.5 px       >1.0 px
Baseline Error         <0.5 mm     <1.0 mm       >1.0 mm
Valid Image Pairs      >=30        >=30          <30
Pattern Detection Rate >95%        >85%          <80%
```

---

## Configuration Examples

### Example 1: Standard ChArUco 7x10 Calibration

```yaml
# Perfect standard calibration result
calibration_date: "2025-01-04T15:30:00"

pattern_config:
  type: "charuco"
  rows: 7
  cols: 10
  square_size_mm: 24.0
  aruco_marker_size_mm: 17.0
  aruco_dictionary: "DICT_4X4_250"

camera_matrix_left:
  fx: 1220.5
  fy: 1221.2
  cx: 640.1
  cy: 360.3

camera_matrix_right:
  fx: 1219.8
  fy: 1220.1
  cx: 638.7
  cy: 359.8

distortion_coeffs_left:
  k1: -0.15
  k2: 0.08
  p1: 0.0009
  p2: -0.0018
  k3: -0.02

distortion_coeffs_right:
  k1: -0.14
  k2: 0.07
  p1: 0.0011
  p2: -0.0015
  k3: -0.018

rotation_matrix:
  - [0.9998, -0.0001, 0.0201]
  - [0.0001, 1.0000, 0.0003]
  - [-0.0201, -0.0003, 0.9998]

translation_vector:
  tx: -70.12
  ty: 0.15
  tz: -0.08

rms_reprojection_error: 0.28
mean_reprojection_error_left: 0.26
mean_reprojection_error_right: 0.29
max_reprojection_error: 0.87

mean_epipolar_error: 0.15
max_epipolar_error: 0.42

baseline_mm: 70.12
baseline_expected_mm: 70.0
baseline_error_mm: 0.12
baseline_error_percent: 0.17

quality_passed: true
rms_check: "PASS"
baseline_check: "PASS"
epipolar_check: "PASS"

num_image_pairs: 50
valid_image_pairs: 48
```

**Analysis:**
- RMS error 0.28px: PASS (excellent)
- Baseline error 0.12mm: PASS (excellent)
- Epipolar error 0.15px: PASS (excellent)
- 48/50 valid pairs: PASS (96% success)
- Overall: Production-ready calibration

### Example 2: Acceptable Calibration with Warnings

```yaml
calibration_date: "2025-01-05T09:15:00"

pattern_config:
  type: "charuco"
  rows: 7
  cols: 10
  square_size_mm: 24.0
  aruco_marker_size_mm: 17.0

# ... intrinsics similar to Example 1 ...

rms_reprojection_error: 0.52
mean_reprojection_error_left: 0.48
mean_reprojection_error_right: 0.56
max_reprojection_error: 1.23

mean_epipolar_error: 0.38
max_epipolar_error: 0.89

baseline_mm: 70.45
baseline_error_mm: 0.45

quality_passed: true
rms_check: "WARNING"        # 0.52 px (0.3-0.6 range)
baseline_check: "PASS"      # 0.45mm (<0.5mm)
epipolar_check: "PASS"

num_image_pairs: 50
valid_image_pairs: 35

warnings:
  - "RMS error 0.52px above recommended (target: 0.3px)"
  - "Only 35/50 valid pairs (70% - acceptable but could be better)"
```

**Analysis:**
- RMS error 0.52px: WARNING (0.3-0.6 range)
- Can be used for production, but quality could be improved
- Recapture with better positioning variety recommended

### Example 3: Failed Calibration

```yaml
calibration_date: "2025-01-05T10:30:00"

# ...

rms_reprojection_error: 0.89
baseline_mm: 72.05
baseline_error_mm: 2.05
baseline_error_percent: 2.9

valid_image_pairs: 28  # Below minimum!

quality_passed: false
rms_check: "FAIL"
baseline_check: "FAIL"

errors:
  - "RMS error 0.89px EXCEEDS limit (max: 0.3px)"
  - "Baseline error 2.05mm CRITICAL (max: 0.5mm)"
  - "Insufficient valid image pairs: 28 (need >= 30)"

warnings:
  - "Pattern may have incorrect dimensions"
  - "Check that square size is exactly 24mm with calipers"
  - "Consider recapturing with better technique"
```

**Analysis:**
- RMS error 0.89px: FAIL (>0.6px)
- Baseline error 2.05mm: FAIL (>1.0mm)
- Valid pairs 28: FAIL (<30)
- **Action Required**: Do not use, recalibrate
- Most likely cause: Incorrect pattern dimensions
- Solution: Verify square size with calipers, recalibrate

---

## Technical References

### Reading YAML Calibration Files

**Python:**
```python
import yaml

with open('/unlook_calib/default.yaml', 'r') as f:
    calib = yaml.safe_load(f)

fx = calib['camera_matrix_left']['fx']
baseline_mm = calib['translation_vector']['tx']
rms_error = calib['rms_reprojection_error']
```

**C++:**
```cpp
cv::FileStorage fs("/unlook_calib/default.yaml",
                   cv::FileStorage::READ);

double fx = (double)fs["camera_matrix_left"]["fx"];
double baseline = (double)fs["translation_vector"]["tx"];
double rms = (double)fs["rms_reprojection_error"];

fs.release();
```

### Computing Depth from Disparity

**Formula:**
```
Z = f * B / d

Where:
- Z: Depth in mm
- f: Focal length in pixels (fx ≈ fy)
- B: Baseline in mm (70.0)
- d: Disparity in pixels (x_left - x_right)
```

**Example Code (C++):**
```cpp
double fx = 1220.5;  // pixels
double baseline_mm = 70.0;

// Disparity of 40 pixels
int disparity = 40;

// Compute depth
double depth_mm = (fx * baseline_mm) / disparity;
double depth_m = depth_mm / 1000.0;

// depth_m ≈ 2.135 meters
```

---

**Document Version**: 1.0
**Last Updated**: 2025-01-04
**Status**: Production Ready
