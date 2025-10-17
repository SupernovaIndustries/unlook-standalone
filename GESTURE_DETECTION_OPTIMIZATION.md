# GESTURE DETECTION OPTIMIZATION GUIDE

**Date**: 2025-10-16
**Context**: Improving hand detection confidence and gesture recognition quality for Unlook 3D Scanner

---

## CURRENT STATUS

### Problems Identified from Logs (2025-10-16_14-23-04)

1. **Hand Detection Working BUT Confidence EXTREMELY LOW**
   - Current scores: 2-3% (0.025-0.033)
   - Required threshold: 70% (0.700)
   - Actual threshold in code: 0.01 (DEBUG mode)
   - **Detection only works because threshold was lowered to 1%**

2. **CRITICAL ONNX Runtime Error**
   ```
   [ERROR] HandLandmarkExtractor: ONNX Runtime error - At least one output should be requested.
   ```
   - **Root Cause**: Fix with `nullptr, 0` does NOT work with ONNX Runtime
   - **Impact**: Landmark extraction COMPLETELY BROKEN
   - **Current State**: Only bbox tracking works, no 21-point landmarks

3. **Gesture Detection Partially Working**
   - SwipeBackward detected with confidence=1
   - Only works because uses bbox centroid tracking
   - Landmarks NOT used (they fail with ONNX error)
   - Quality degraded without landmark-based tracking

4. **Detection Quality Issues**
   - Multiple false positives (7-8 detections per frame)
   - NMS reduces to 1 detection but confidence remains low
   - Bbox stable but confidence unstable

---

## IMMEDIATE CRITICAL FIX - ONNX Runtime

### Problem
The fix using `nullptr, 0` for output_names in ONNX Runtime does NOT work:
```cpp
// BROKEN CODE (HandLandmarkExtractor.cpp:346-356)
auto output_tensors = pImpl->ort_session->Run(
    Ort::RunOptions{nullptr},
    pImpl->input_names.data(),
    &input_tensor_ort, 1,
    nullptr,  // ❌ CAUSES ERROR: "At least one output should be requested"
    0
);
```

### Solution
ONNX Runtime REQUIRES at least one output name to be specified. The fix is to use the output_names array that was already collected during initialization:

```cpp
// CORRECT CODE
auto output_tensors = pImpl->ort_session->Run(
    Ort::RunOptions{nullptr},
    pImpl->input_names.data(),
    &input_tensor_ort, 1,
    pImpl->output_names.data(),  // ✅ Use the names we collected
    pImpl->output_names.size()   // ✅ All 3 outputs (xyz_x21s, hand_scores, handedness)
);
```

**Why This Works**:
- During initialization (lines 130-139), we already collect ALL output names into `output_name_strs`
- We build the `output_names` vector with pointers to these strings
- ONNX Runtime expects valid output names, not nullptr
- The original name-matching issue was likely due to encoding problems or whitespace
- The names are extracted directly from the model metadata, so they MUST match

**Implementation Priority**: CRITICAL - This must be fixed FIRST before any other optimizations

---

## ROOT CAUSE ANALYSIS - Low Confidence Scores

### Why Confidence is 2-3% Instead of 70%+

Based on research and log analysis, possible causes:

#### 1. **Model Input Mismatch**
- **PINTO0309 models expect specific preprocessing**:
  - Normalization: [0, 1] range (divide by 255.0)
  - Color space: RGB (NOT BGR)
  - Input size: Exactly 192x192 for palm detection
  - Format: CHW (channels first)

- **Current Implementation** (HandDetector.cpp:171-211):
  ```cpp
  resized.convertTo(resized, CV_32F, 1.0 / 255.0);  // ✅ Normalization OK
  cv::cvtColor(resized, resized, cv::COLOR_BGR2RGB); // ✅ RGB conversion OK
  // HWC → CHW conversion using cv::split ✅ OK
  ```
  - **Preprocessing appears CORRECT**

#### 2. **Camera Color Space Issue**
- **Current camera capture**: YUV420 format from libcamera
- **Conversion chain**: YUV420 → BGR → RGB → Float normalization
- **Problem**: YUV→BGR conversion may introduce artifacts or color shift

**Research Finding** (from search):
> "YUV, YCrCb, and HSV color spaces have shown better performance than RGB for image classification tasks"
> "YUV separates the Y channel (intensity) from U and V channels to eliminate the effects of luminance"

- **Hypothesis**: Converting YUV420 → BGR → RGB may degrade hand detection
- **Better Approach**: Process in YUV space OR improve YUV→RGB conversion quality

#### 3. **Lighting Conditions**
- **User Report**: "luminosita aumentata ma nulla ancora" (brightness increased but still nothing)
- **Current Setup**: Auto-exposure DISABLED, manual brightness control
- **Problem**: Static exposure may not adapt to hand position/background

**Research Finding** (from search):
> "Proper exposure settings are crucial for modern machine vision cameras to accurately convert light into clear images"
> "Auto Exposure adjusts gain and exposure time to optimize the brightness of the image"

- **Hypothesis**: Fixed exposure causes poor contrast between hand and background
- **Better Approach**: Enable auto-exposure with target brightness optimization

#### 4. **Image Normalization Issues**
- **Current**: Simple division by 255.0
- **Research Finding**:
  > "Splitting and normalizing each channel separately by subtracting the mean and dividing by the standard deviation is a better way to normalize YUV images"

- **Hypothesis**: PINTO0309 models may expect channel-wise normalization
- **Better Approach**: Per-channel mean/std normalization (ImageNet stats?)

#### 5. **Contrast and Dynamic Range**
- **Current**: No contrast enhancement
- **Research Finding**:
  > "CLAHE (Contrast Limited Adaptive Histogram Equalization) is an image processing method that suppresses noise while enhancing the contrast"
  > "Converting images to YUV and performing histogram equalization on the Y channel can normalize the luminance"

- **Hypothesis**: Low contrast between hand and background reduces detection confidence
- **Better Approach**: Apply CLAHE or histogram equalization preprocessing

---

## OPTIMIZATION STRATEGIES

### Priority 1: FIX ONNX Runtime (CRITICAL)

**File**: `src/gesture/HandLandmarkExtractor.cpp`
**Line**: 346-356

**Change**:
```cpp
// BEFORE (BROKEN):
auto output_tensors = pImpl->ort_session->Run(
    Ort::RunOptions{nullptr},
    pImpl->input_names.data(),
    &input_tensor_ort, 1,
    nullptr,  // ❌ ERROR
    0
);

// AFTER (FIXED):
auto output_tensors = pImpl->ort_session->Run(
    Ort::RunOptions{nullptr},
    pImpl->input_names.data(),
    &input_tensor_ort, 1,
    pImpl->output_names.data(),  // ✅ Use collected names
    pImpl->output_names.size()
);
```

**Expected Result**: Landmark extraction should work, logs will show "SUCCESS - 21 landmarks extracted"

---

### Priority 2: Enable Auto-Exposure

**File**: `src/gui/GestureWidget.cpp`
**Current**: Auto-exposure is disabled per user request
**Problem**: Fixed exposure causes poor hand/background contrast

**Proposed Change**:
```cpp
// Add auto-exposure toggle in GUI
bool use_auto_exposure = true;  // Enable by default for gesture detection

if (use_auto_exposure) {
    camera_system->enableAutoExposure(true);
    camera_system->setTargetBrightness(128);  // Mid-range target
} else {
    camera_system->enableAutoExposure(false);
    // Manual exposure/gain settings
}
```

**Benefits**:
- Automatically adapts to hand position and lighting
- Optimizes contrast between hand and background
- Research shows 1ms convergence time for deep RL-based auto-exposure
- Should significantly improve confidence scores

**User Preference**: "no niente auto exposure per ora" - but may need to reconsider for gesture detection

---

### Priority 3: Improve Color Space Handling

**Current Chain**: YUV420 → BGR → RGB → Normalization

**Option A: Direct YUV Processing** (Research-Backed)
```cpp
// In HandDetector.cpp preprocessing
// Instead of YUV→BGR→RGB, process YUV directly:

cv::Mat yuv_image;
cv::cvtColor(image, yuv_image, cv::COLOR_BGR2YUV);  // Assuming input is BGR

// Apply CLAHE on Y channel for contrast enhancement
std::vector<cv::Mat> yuv_channels;
cv::split(yuv_image, yuv_channels);

cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(2.0, cv::Size(8, 8));
clahe->apply(yuv_channels[0], yuv_channels[0]);

cv::merge(yuv_channels, yuv_image);

// Convert to RGB for model
cv::Mat rgb_image;
cv::cvtColor(yuv_image, rgb_image, cv::COLOR_YUV2RGB);

// Then normalize as before
```

**Benefits**:
- Better handling of luminance variations
- CLAHE improves contrast without over-amplification
- YUV color space is more robust to lighting changes

**Option B: Improved YUV→RGB Conversion**
```cpp
// Use different color conversion flags
cv::cvtColor(image, rgb_image, cv::COLOR_YUV420p2RGB);  // More accurate conversion

// Or if input is already processed:
cv::cvtColor(image, rgb_image, cv::COLOR_YUV2RGB_I420);
```

---

### Priority 4: Channel-Wise Normalization

**Current**: Simple `/255.0` normalization
**Research-Backed**: Per-channel mean/std normalization

**File**: `src/gesture/HandDetector.cpp`, `src/gesture/HandLandmarkExtractor.cpp`

**Implementation**:
```cpp
// After converting to float and dividing by 255.0
// Apply ImageNet normalization (common for many models)
const float mean[3] = {0.485f, 0.456f, 0.406f};  // ImageNet RGB means
const float std[3] = {0.229f, 0.224f, 0.225f};   // ImageNet RGB stds

// In CHW format, normalize each channel
size_t channel_size = config.input_height * config.input_width;
for (int c = 0; c < 3; ++c) {
    float* channel_data = input_tensor.data() + c * channel_size;
    for (size_t i = 0; i < channel_size; ++i) {
        channel_data[i] = (channel_data[i] - mean[c]) / std[c];
    }
}
```

**Note**: Need to verify if PINTO0309 models use ImageNet stats or [0,1] normalization. Check model documentation or reference Python implementation.

---

### Priority 5: Adjust Confidence Threshold Strategy

**Current Approach**: Lowered threshold to 0.01 (1%) to get ANY detections
**Problem**: This is a workaround, not a solution

**Better Strategy** (Based on Research):
> "When not seeing predictions, try lowering the confidence score as the threshold may be too high"
> "A high threshold ensures only the most certain predictions are accepted, while a lower threshold allows for more detections but increases the risk of false positives"

**Proposed Approach**:
```cpp
// Instead of fixed threshold, use adaptive thresholding
HandDetectorConfig config;

// Stage 1: Lower threshold to 0.05 (5%) initially
config.confidence_threshold = 0.05f;

// Stage 2: After getting detections working, gradually increase
// Once preprocessing is fixed, target 0.3-0.5 (30-50%) as reasonable threshold
// Production target: 0.6-0.7 (60-70%) after all optimizations

// Use temporal consistency to filter false positives:
// - Track detection history over multiple frames
// - Only accept detections that appear in 3+ consecutive frames
// - This compensates for lower confidence threshold
```

---

### Priority 6: Model-Specific Optimizations

**Check PINTO0309 Reference Implementation**:
1. Download reference Python demo from: https://github.com/PINTO0309/hand-gesture-recognition-using-onnx
2. Compare preprocessing pipeline step-by-step
3. Verify normalization values
4. Check for any post-processing on detection scores

**Potential Issues to Verify**:
- Score calibration (some models output logits, need sigmoid/softmax)
- NMS parameters (IoU threshold may be filtering good detections)
- Input size expectations (192x192 strict requirement)
- Aspect ratio handling (letterboxing vs. stretching)

---

## TESTING PROTOCOL

### Phase 1: Fix ONNX Runtime
1. Apply ONNX output_names fix
2. Rebuild and test
3. **Success Criteria**: Logs show "SUCCESS - 21 landmarks extracted"

### Phase 2: Baseline Measurement
1. Keep current settings (auto-exposure OFF)
2. Record confidence scores over 100 frames
3. Document: min/max/avg confidence, detection rate

### Phase 3: Enable Auto-Exposure
1. Enable auto-exposure with target_brightness=128
2. Record confidence scores over 100 frames
3. Compare with baseline

### Phase 4: Add CLAHE Preprocessing
1. Implement YUV CLAHE preprocessing
2. Record confidence scores over 100 frames
3. Compare with previous phases

### Phase 5: Channel-Wise Normalization
1. Add ImageNet mean/std normalization
2. Record confidence scores over 100 frames
3. Compare with previous phases

### Phase 6: Threshold Optimization
1. With all preprocessing fixes, test thresholds: 0.3, 0.4, 0.5, 0.6, 0.7
2. Find optimal balance between detection rate and false positives

---

## EXPECTED RESULTS

### After ONNX Fix
- Landmark extraction working
- Gesture detection using 21-point tracking (more accurate than bbox only)
- Better gesture classification with finger positions

### After Auto-Exposure
- Confidence scores increase from 2-3% to 10-30%
- More consistent detection across different hand positions
- Better adaptation to background changes

### After CLAHE + YUV Processing
- Confidence scores increase to 30-50%
- Better performance in varying lighting
- Reduced false positives

### After Channel-Wise Normalization
- Confidence scores reach 50-70%+ (production target)
- Stable detection with minimal false positives
- Gesture detection reliable enough for production

---

## IMPLEMENTATION PRIORITY

1. **IMMEDIATE**: Fix ONNX Runtime (HandLandmarkExtractor.cpp:346-356)
2. **HIGH**: Enable auto-exposure (may require user approval)
3. **MEDIUM**: Add CLAHE preprocessing on Y channel
4. **MEDIUM**: Implement channel-wise normalization
5. **LOW**: Verify against PINTO0309 reference implementation
6. **ONGOING**: Iterative threshold tuning after each optimization

---

## ALTERNATIVE APPROACHES (If Above Fails)

### 1. Switch to Different Model Variant
- PINTO0309 provides multiple model sizes: 128x128, 192x192, 256x256
- Larger models may have better accuracy but slower inference
- Try 256x256 variant if 192x192 continues to have low confidence

### 2. MediaPipe C++ Official SDK
- Google's official C++ implementation with proven performance
- Pre-integrated preprocessing pipeline
- Better documentation and community support
- See: `GESTURE_RECOGNITION_RESEARCH.md` for details

### 3. Fine-Tune Model on Custom Dataset
- Collect 1000+ images of hands in actual usage conditions
- Fine-tune PINTO0309 model or train custom model
- Optimize for specific lighting, camera, and background conditions
- **Most effort but highest quality potential**

---

## REFERENCES

### Research Sources
1. PINTO0309 hand-gesture-recognition-using-onnx: https://github.com/PINTO0309/hand-gesture-recognition-using-onnx
2. PINTO_model_zoo Hand Detection: https://github.com/PINTO0309/PINTO_model_zoo/tree/main/033_Hand_Detection_and_Tracking
3. YUV vs RGB for Human-Machine Interaction (ResearchGate)
4. CLAHE for Image Preprocessing (MDPI, Nature Communications)
5. Auto-Exposure Algorithms for Machine Vision (2024-2025 research)
6. ONNX Runtime C++ API Documentation (Microsoft)

### Key Findings
- YUV color space more robust for varying lighting
- CLAHE improves contrast without noise amplification
- Per-channel normalization improves model accuracy
- Auto-exposure critical for hand detection in dynamic scenes
- PINTO0309 models expect [0,1] normalization with RGB input

---

**Last Updated**: 2025-10-16
**Next Steps**: Fix ONNX Runtime, then enable auto-exposure for testing
