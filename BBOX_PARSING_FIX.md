# BBOX PARSING FIX - PINTO0309 Format Correction

## Problem Identified

The bounding box parsing in `HandDetector.cpp` was using the **wrong format** for PINTO0309's palm detection model output.

### Root Cause

The code was treating the model output as if it had 18+ features (MediaPipe full format), when in reality it outputs **8 features** in PINTO0309's simplified format.

### Evidence from Python Reference

File: `/home/alessandro/unlook-gesture/third-party/hand-gesture-recognition-using-onnx/model/palm_detection/palm_detection.py`

**Lines 195-196, 212**:
```python
# boxes: np.ndarray
#     float32[N, 8]
#     pd_score, box_x, box_y, box_size, kp0_x, kp0_y, kp2_x, kp2_y

pd_score, box_x, box_y, box_size, kp0_x, kp0_y, kp2_x, kp2_y = box
```

### PINTO0309 Format (8 features)

```
[0] = pd_score      (detection confidence score, [0,1])
[1] = box_x         (center X coordinate, normalized [0,1])
[2] = box_y         (center Y coordinate, normalized [0,1])
[3] = box_size      (SINGLE size value - bounding box is SQUARE!)
[4] = kp0_x         (keypoint 0 X, normalized [0,1])
[5] = kp0_y         (keypoint 0 Y, normalized [0,1])
[6] = kp2_x         (keypoint 2 X, normalized [0,1])
[7] = kp2_y         (keypoint 2 Y, normalized [0,1])
```

**Critical Note**: The bounding box is **SQUARE** - `box_size` is used for both width and height!

## Original Bug (WRONG CODE)

```cpp
// Lines ~279-282 (BEFORE FIX)
if (num_features >= 18) {
    cx = detection[0] * config.input_width * scale_x;   // ❌ detection[0] is SCORE!
    cy = detection[1] * config.input_height * scale_y;  // ❌ detection[1] is box_x!
    w = detection[2] * config.input_width * scale_x;    // ❌ detection[2] is box_y!
    h = detection[3] * config.input_height * scale_y;   // ❌ detection[3] is box_size!
}
```

### Consequences

- **Score was used as center X** → Random bbox positions
- **box_x was used as center Y** → Completely wrong coordinates
- **box_y was used as width** → Bboxes 1-2 pixels wide (as seen in logs)
- **box_size was used as height** → Tiny bboxes that missed hands

### Log Evidence

```
Output[0] shape: [312, 8]  ← 8 features, NOT 18!
Detection[0]: score=0.022046
Detection[0]: bbox=[5, 0, 1, 4]  ← Width=1px, Height=4px (WRONG!)
```

## Fix Applied

### Corrected Parsing Logic

```cpp
// Lines 229-305 (AFTER FIX)
if (num_features == 8) {
    // PINTO0309 palm detection format
    // [0] = pd_score (detection confidence)
    score = detection[0];

    // Parse bounding box
    float box_x = detection[1];     // center X normalized [0,1]
    float box_y = detection[2];     // center Y normalized [0,1]
    float box_size = detection[3];  // square box size normalized [0,1]

    // Convert normalized coordinates to pixel coordinates
    cx = box_x * config.input_width * scale_x;
    cy = box_y * config.input_height * scale_y;
    w = box_size * config.input_width * scale_x;
    h = box_size * config.input_height * scale_y;  // Same as width (square bbox!)
}
```

### Key Changes

1. **Explicit `num_features == 8` check** for PINTO0309 format
2. **Score from `detection[0]`** (correct position)
3. **Bounding box from `detection[1-3]`**:
   - `detection[1]` = box_x (center X)
   - `detection[2]` = box_y (center Y)
   - `detection[3]` = box_size (used for BOTH width and height)
4. **Proper normalization**: Multiply by `input_width/height` then by `scale_x/y`
5. **Square bbox**: Same value for width and height

### Additional Improvements

- **Diagnostic logging** for raw PINTO0309 values (first 5 detections)
- **Preserved fallback logic** for MediaPipe full format (18+ features)
- **Clear comments** referencing Python source file

## Expected Results After Fix

### Before Fix
```
Detection[0]: bbox=[5, 0, 1, 4]     ← Tiny bboxes (1-2 pixels)
NO DETECTIONS FOUND!
```

### After Fix (Expected)
```
Detection[0]: bbox=[200, 300, 150, 150]  ← Realistic hand-sized bboxes!
Raw PINTO0309 values: [score=0.85, box_x=0.52, box_y=0.61, box_size=0.39]
HAND DETECTED: score=0.85, bbox=[200, 300, 150, 150]
```

## Testing Checklist

- [x] Build succeeds without errors
- [ ] Run `gesture_test_simple` with test image
- [ ] Run `gesture_test_camera` with real camera
- [ ] Verify bboxes are 100-300 pixels (realistic hand size)
- [ ] Verify detection scores improve when hand is visible
- [ ] Check diagnostic logs show correct raw values

## Files Modified

- `/home/alessandro/unlook-gesture/src/gesture/HandDetector.cpp`
  - Lines 222-331: Parsing logic completely rewritten
  - Score extraction: Now handles 8-feature format correctly
  - Bbox parsing: Implements PINTO0309 format with square bboxes
  - Diagnostic logging: Added raw value logging for debugging

## References

- **Python reference**: `third-party/hand-gesture-recognition-using-onnx/model/palm_detection/palm_detection.py`
- **Model**: `palm_detection_full_inf_post_192x192.onnx`
- **Format documentation**: PINTO0309 MediaPipe palm detection (8-feature output)

## Next Steps

1. **Test with gesture_test_simple**: Verify bbox parsing with static image
2. **Test with gesture_test_camera**: Validate real-time hand detection
3. **Monitor logs**: Check for realistic bbox sizes (100-300px)
4. **Tune threshold**: If needed, adjust score threshold for optimal detection

---

**Status**: FIX APPLIED AND BUILT ✅
**Build**: Successful (October 16, 2025)
**Ready for Testing**: YES
