# SGBM Competitor Analysis - Luxonis OakD vs Intel RealSense vs Unlook

**Data:** 2025-11-14
**Purpose:** Compare stereo matching best practices from industrial competitors

---

## 1. Luxonis OakD (DepthAI)

### Algorithm
- **Cost Aggregation:** Semi-Global Block Matching (SGBM)
- **Disparity Levels:** 96 standard, 191 with Extended Disparity mode
- **Baseline:** 7.5cm (OakD), 9cm (OakD Lite), 15cm (OakD Pro W)

### Key Parameters

| Parameter | Default/Recommended | Notes |
|-----------|---------------------|-------|
| **Median Filter** | 7x7 kernel | Disable for subpixel mode 4-5 bits |
| **Subpixel Mode** | 3 fractional bits | 8x granularity, suitable for long range |
| **LR-Check** | Enabled | Better occlusion handling |
| **Confidence Threshold** | 200 | Lower = higher confidence required |
| **Extended Disparity** | For short-range | Increases levels to 191 (from 96) |

### Post-Processing Filters
1. **Temporal Filter:** Improves persistency using previous frames
2. **Spatial Filter:** Edge-preserving, fills invalid pixels
3. **Speckle Filter:** Reduces speckle noise
4. **Decimation Filter:** Sub-samples depth map for performance

### Best Practices
- **High IQ stereo images prerequisite**
- **Longer exposure instead of higher ISO** for low-light (better SNR)
- **Subpixel mode** especially if object isn't close to MinZ
- **Low confidence threshold** for best accuracy
- **Median filter 7x7** for passive stereo (hardware accelerated)

---

## 2. Intel RealSense D435

### Algorithm
- **Cost Matching:** SSD (Sum of Squared Differences) block-matching
- **Proprietary:** Details not publicly available
- **Advanced Mode:** >40 configurable parameters (machine learning optimized)

### Key Parameters

| Parameter | Default/Recommended | Notes |
|-----------|---------------------|-------|
| **Resolution** | 848x480 @ 30fps | With auto-exposure |
| **Texture Count Threshold** | 0-4 | Evidence of texture required |
| **Texture Difference Threshold** | Variable | How big a difference = texture |
| **DSSecondPeakThreshold** | Variable | Ambiguity detection (2nd best match) |
| **Depth Clamp Min/Max** | Variable | Restrict scanned depth range |

### Visual Presets
- **High Accuracy:** High confidence, lower fill factor
- **High Density:** Higher fill factor, sees more objects
- **Medium Density:** Balance between fill and accuracy
- **Default:** Best visual appeal, clean edges

### Best Practices
- **Start from recommended defaults:** 848x480 @ 30fps
- **Use Visual Presets** as starting point
- **Machine learning optimization** via Intel's presets
- **JSON configuration files** for advanced tuning
- **Complex parameter interplay:** Use presets instead of manual tuning

---

## 3. Unlook Scanner (Current Implementation)

### Algorithm
- **Cost Matching:** SGM-Census (Semi-Global Matching with Census Transform)
- **Census Window:** 9x9 = 80-bit descriptor (exact match to epiception/SGM-Census)
- **Disparity Levels:** 384 (more than both competitors!)

### Current Parameters

| Parameter | Current Value | Notes |
|-----------|---------------|-------|
| **Census Window** | 9x9 (80 bits) | EXACT match to epiception repo |
| **P1 (small jump)** | 8 | Standard SGM penalty |
| **P2 (large jump)** | 32 | Standard SGM penalty |
| **Disparity Range** | 0-384 pixels | 2x more than OakD! |
| **Uniqueness Check** | 15% margin | Similar to RealSense DSSecondPeakThreshold |
| **Vertical Search** | Disabled (0px) | Was ±8px, testing showed no improvement |
| **Subpixel** | CV_16S (×16) | 16x subpixel precision |

### Post-Processing
- **WLS Filter:** Available (λ=8000, σ=1.5)
- **Outlier Rejection:** Sigma-based (2.5σ threshold)
- **Multi-frame Fusion:** Median of inliers

### Hardware
- **Resolution:** 1280x720 (higher than RealSense recommended)
- **Baseline:** 69.4mm (between OakD and OakD Pro W)
- **Sensor:** IMX296 Global Shutter
- **VCSEL:** 2× BELAGO 1.2 (30k dots total)

---

## 4. Comparison Matrix

| Feature | Luxonis OakD | Intel RealSense D435 | Unlook Scanner |
|---------|--------------|----------------------|----------------|
| **Algorithm** | SGBM | SSD Block-Matching | SGM-Census |
| **Disparity Levels** | 96 (191 extended) | Proprietary | **384** ✓ |
| **Baseline** | 75-150mm | 50mm | 69.4mm |
| **Subpixel** | 3 bits (8x) | Yes | **4 bits (16x)** ✓ |
| **Post-Processing** | 4 filters (HW) | Advanced Mode | WLS + Outlier + Fusion |
| **Uniqueness Check** | LR-Check | DSSecondPeakThreshold | **15% margin** ✓ |
| **Median Filter** | 7x7 HW | Unknown | Not implemented ❌ |
| **Temporal Filter** | Yes (HW) | Yes | **Multi-frame fusion** ✓ |
| **Resolution** | 1280x800 max | 1280x720 max | 1280x720 |
| **VCSEL/IR** | Optional | IR projector | **2× VCSEL (30k dots)** ✓ |

---

## 5. Recommended Improvements for Unlook

Based on competitor best practices:

### HIGH PRIORITY ✅
1. **✅ Census Transform (9x9)** - Already implemented correctly
2. **✅ Uniqueness Check (15%)** - Already implemented
3. **✅ Multi-frame Fusion** - Already implemented (median of inliers)
4. **✅ WLS Filter** - Already available
5. **✅ Extended Disparity Range (384)** - Already 2x better than OakD!

### MEDIUM PRIORITY 🔄
1. **🔄 Median Filter (7x7)** - Add post-SGM median filter for noise reduction
   - Luxonis uses hardware-accelerated 7x7 kernel
   - Can be implemented in software with OpenCV

2. **🔄 Temporal Filter** - Enhance multi-frame fusion with temporal weighting
   - Currently: median of inliers across frames
   - Improvement: weighted by temporal distance + confidence

3. **🔄 Speckle Filter** - Add dedicated speckle noise reduction
   - OpenCV provides `filterSpeckles()` function
   - Removes isolated disparity regions

### LOW PRIORITY (OPTIMIZATION) 🔮
1. **🔮 Automatic Parameter Tuning** - Implement visual presets like RealSense
   - Preset 1: High Accuracy (higher P1/P2, strict uniqueness)
   - Preset 2: High Density (lower P1/P2, relaxed uniqueness)
   - Preset 3: VCSEL Optimized (optimized for 30k dot patterns)

2. **🔮 Texture Detection** - Add texture-based confidence like RealSense
   - Reject matches in textureless regions
   - Improves reliability in uniform surfaces

3. **🔮 Decimation Filter** - For performance optimization
   - Sub-sample depth map before filtering
   - 2x faster processing with minimal quality loss

---

## 6. Current Issues Analysis

### Issue: Cone/Frustum Pattern (SOLVED)

**Root Cause:** Rectification broken (49.4px epipolar error, 521px max)

**Competitor Comparison:**
- Luxonis max_epipolar_error: Not specified (assumed <2px from good rectification)
- RealSense: Factory calibrated to <1px epipolar error
- **Unlook current:** 49.4px average ❌ (CRITICAL BUG)

**Fix Applied:** Swap rectification L↔R (camera order mismatch)

**Expected After Fix:**
- Epipolar error: <2px ✓
- Disparity: Real object structure (not cone) ✓
- Census matching: Correct correspondences ✓

---

## 7. Conclusions

### Unlook Strengths
1. **✅ Higher disparity range (384 vs 96/191)** - Better precision
2. **✅ Census Transform (80-bit)** - Robust to illumination changes
3. **✅ VCSEL structured light (30k dots)** - Texture in textureless surfaces
4. **✅ Higher subpixel precision (16x vs 8x)** - Sub-millimeter depth

### Areas for Improvement
1. **❌ Epipolar alignment (CRITICAL)** - Must be <2px (currently 49.4px)
2. **🔄 Median filter** - Add 7x7 post-processing like OakD
3. **🔄 Speckle filter** - Reduce noise in uniform regions
4. **🔮 Visual presets** - Implement automatic tuning modes

### Next Steps
1. **VERIFY rectification swap fix** - Test scan after L↔R swap
2. **If fixed:** Implement median + speckle filters
3. **If NOT fixed:** Full recalibration required
4. **Long-term:** Implement visual presets (High Accuracy, High Density, VCSEL Optimized)

---

**Generated:** 2025-11-14
**Sources:**
- https://docs.luxonis.com/projects/api/en/latest/tutorials/configuring-stereo-depth/
- https://github.com/IntelRealSense/librealsense/wiki/D400-Series-Visual-Presets
- https://dev.realsenseai.com/docs/tuning-depth-cameras-for-best-performance
