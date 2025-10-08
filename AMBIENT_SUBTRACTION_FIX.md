# Ambient Subtraction Fix - Weighted Subtraction Implementation

## Problem Statement
The original ambient subtraction in the Unlook 3D scanner was removing the foreground subject (hand) instead of just the background noise. This occurred because subjects reflect significant ambient light, causing full subtraction to eliminate them from the image.

## Solution Implemented
Replaced full subtraction with **weighted subtraction** to preserve the subject while reducing ambient noise.

### Key Changes

#### 1. Weighted Subtraction Algorithm
Instead of:
```cpp
cv::subtract(frame1.left_frame.image, frame3.left_frame.image, pattern1_left);
```

Now using:
```cpp
cv::addWeighted(frame1.left_frame.image, 1.0, frame3.left_frame.image, -0.3, 0, pattern1_left);
pattern1_left.convertTo(pattern1_left, -1, 1.5, 0);  // Apply gain boost
```

#### 2. Configurable Parameters
Added two new member variables to `DepthTestWidget`:
- `ambient_subtraction_weight_`: Controls how much ambient to subtract (default: 0.3 = 30%)
- `ambient_pattern_gain_`: Gain boost after subtraction (default: 1.5)

#### 3. Public API for Adjustment
Added methods to dynamically adjust parameters:
```cpp
void setAmbientSubtractionWeight(float weight);  // 0.0 to 1.0
void setAmbientPatternGain(float gain);          // 0.5 to 3.0
```

## Technical Details

### Weight Parameter
- **0.0**: No ambient subtraction (original image)
- **0.3** (default): Subtract 30% of ambient - preserves subject while reducing noise
- **1.0**: Full subtraction (original behavior - removes subject)

### Gain Compensation
After weighted subtraction, the image brightness is reduced. A gain of 1.5 compensates for this reduction while maintaining VCSEL dot visibility.

### Mathematical Operation
```
Result = VCSEL_frame * 1.0 + Ambient_frame * (-weight)
Final = Result * gain
```

## Files Modified
1. `/home/alessandro/unlook-standalone/src/gui/depth_test_widget.cpp`
   - Lines 285-320: Implemented weighted subtraction
   - Lines 41-42: Added parameter initialization
   - Lines 97-102: Updated debug logging

2. `/home/alessandro/unlook-standalone/include/unlook/gui/depth_test_widget.hpp`
   - Lines 275-276: Added member variables
   - Lines 159-173: Added parameter adjustment methods

## Testing Recommendations
1. Start with default weight of 0.3 (30% subtraction)
2. If subject still disappears, reduce to 0.2 or 0.15
3. If too much ambient remains, increase to 0.4 or 0.5
4. Adjust gain if image is too dark (increase) or saturated (decrease)

## Benefits
- Preserves foreground subjects (hands, objects)
- Maintains VCSEL dot pattern brightness
- Reduces ambient noise effectively
- Configurable for different lighting conditions
- No negative value clipping issues

## Integration Notes
- The existing ambient subtraction checkbox remains functional
- When enabled, uses weighted subtraction with configured parameters
- When disabled, uses original VCSEL frames without any subtraction
- Debug logs show current weight and gain values for monitoring