# Gesture Recognition Test Programs

This directory contains test programs for the Unlook gesture recognition system.

## Available Test Programs

### 1. gesture_test_simple (READY TO USE ✅)

**Status**: Compiled and ready to test
**Location**: `build/examples/gesture_test_simple`
**Dependencies**: OpenCV VideoCapture (standard webcam/video support)

Simple standalone test program that uses OpenCV's VideoCapture for input.
Perfect for testing gesture recognition algorithms without hardware dependencies.

#### Usage

```bash
# Basic usage with default webcam (camera 0)
./build/examples/gesture_test_simple

# Use specific camera
./build/examples/gesture_test_simple --camera 1

# Process video file
./build/examples/gesture_test_simple --video /path/to/video.mp4

# Adjust confidence threshold
./build/examples/gesture_test_simple --confidence 0.6

# Enable debug visualization
./build/examples/gesture_test_simple --debug

# Full options
./build/examples/gesture_test_simple --help
```

#### Command Line Options

- `--camera <id>` - Camera device ID (default: 0)
- `--video <file>` - Use video file instead of camera
- `--confidence <val>` - Minimum gesture confidence threshold (0.0-1.0, default: 0.7)
- `--debug` - Enable debug visualization
- `--help` - Show help message

#### Features

- Real-time gesture detection from webcam or video file
- Live statistics (FPS, processing time, detection count)
- Gesture histogram tracking
- Clean shutdown with CTRL+C
- Comprehensive session report

#### Supported Gestures

- **Swipe LEFT/RIGHT** - Horizontal hand movement
- **Swipe UP/DOWN** - Vertical hand movement
- **Swipe FORWARD** - Hand moving toward camera
- **Swipe BACKWARD** - Hand moving away from camera
- **Open Palm** - Hand fully open
- **Closed Fist** - Hand closed
- **Point Up/Down** - Index finger pointing
- **Thumbs Up/Down** - Thumb gestures

#### Example Output

```
=========================================
   UNLOOK GESTURE RECOGNITION TEST
=========================================
Configuration:
  Camera ID:        0
  Min confidence:   70.0%
  Debug viz:        Disabled
=========================================

Video source opened successfully!
  Resolution: 640x480
  FPS: 30.0

Gesture recognition system ready!
Detecting gestures...

========================================
🎯 GESTURE DETECTED: SwipeRight
========================================
  Confidence:      85.30%
  Processing time: 12.4 ms
  Center position: (320, 240)
  Bounding box:    120x180 px
  Hand:            Right
  Total gestures:  1
========================================

FPS: 28.5 | Frames: 150
Avg processing: 11.8 ms (Detection: 8.2 ms)
```

---

### 2. gesture_test (ORIGINAL - NOT COMPILED)

**Status**: Source available but not compiled due to camera system conflicts
**Location**: `examples/gesture_test.cpp`
**Dependencies**: Unlook SynchronizedCameraSystem

Original version designed to integrate with the Unlook hardware camera system.
Currently disabled due to duplicate symbol errors in camera library.

**Note**: This version will be enabled once the camera system duplicate definition
issues are resolved (camera_system.cpp vs CameraSystem.cpp conflict).

---

## Building

The test programs are built automatically with the main project:

```bash
cd /home/alessandro/unlook-gesture/build
cmake .. -DCMAKE_BUILD_TYPE=Release
make gesture_test_simple -j4
```

## Testing Workflow

### Quick Test (No Hardware)

```bash
# Test with default webcam
./build/examples/gesture_test_simple

# Test with video file
./build/examples/gesture_test_simple --video test_video.mp4
```

### Performance Testing

```bash
# Lower confidence for more detections
./build/examples/gesture_test_simple --confidence 0.5

# Monitor for 30 seconds, check final statistics
# Press CTRL+C when done
```

### Development Testing

```bash
# Enable debug mode (future feature)
./build/examples/gesture_test_simple --debug
```

## Troubleshooting

### "Failed to open video source"

- Check if camera is connected: `v4l2-ctl --list-devices`
- Try different camera ID: `--camera 1` or `--camera 2`
- Check camera permissions: user must be in `video` group

### "Failed to initialize gesture system"

- Verify model files exist in `third-party/hand-gesture-recognition-using-onnx/model/`
- Check paths:
  - `third-party/hand-gesture-recognition-using-onnx/model/palm_detection/palm_detection_full_inf_post_192x192.onnx`
  - `third-party/hand-gesture-recognition-using-onnx/model/hand_landmark/hand_landmark_sparse_Nx3x224x224.onnx`

### Low FPS / High Processing Time

- Reduce camera resolution if possible
- Lower confidence thresholds
- Close other applications using camera/CPU
- Check system load: `htop`

### No Gestures Detected

- Ensure good lighting conditions
- Keep hand clearly visible to camera
- Try adjusting `--confidence` threshold lower (0.5-0.6)
- Verify hand is within camera frame
- Perform clear, deliberate gesture movements

## Next Steps

1. **Test on Raspberry Pi CM5**: Cross-compile and test on target hardware
2. **Integrate with Unlook Camera**: Resolve duplicate symbol errors in camera library
3. **Add GUI Demo**: Visual feedback with live camera preview
4. **Performance Optimization**: Benchmark and optimize for real-time performance

## Library Status

**Gesture Recognition Library**: ✅ Built successfully
**Location**: `build/src/gesture/libunlook_gesture.a` (1.7 MB)

**Components**:
- HandDetector ✅
- HandLandmarkExtractor ✅
- HandTracker ✅
- TemporalBuffer ✅
- GeometricSwipeDetector ✅
- GestureRecognitionSystem ✅

---

**Ready for testing!** Start with `gesture_test_simple` and verify gesture recognition works correctly.
